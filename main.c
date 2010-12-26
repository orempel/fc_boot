/***************************************************************************
 *   Copyright (C) 09/2007 by Olaf Rempel                                  *
 *   razzor@kopf-tisch.de                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; version 2 of the License,               *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

/*
 * ATMega 644P:
 * Fuse E: 0xFD (2.7V BOD)
 * Fuse H: 0xDA (2048 words bootloader)
 * Fuse L: 0xFF (ext. Crystal)
 */

#define F_CPU 20000000
#include <util/delay.h>

#define BAUDRATE 57600
#define UART_CALC_BAUDRATE(baudRate) ((uint32_t)(F_CPU) / ((uint32_t)(baudRate)*16) -1)

#define APP_END			0xF000

#define LED_RT			(1<<PORTB0)			/* low active */
#define LED_GN			(1<<PORTB1)			/* high active */

#define TWI_CLK			100000
#define TWI_ADDRESS_BASE	0x20				/* 0x21 - 0x24 */

#define TWI_DEVCODE		0x77				/* Mega8 */
#define OWN_DEVCODE		0x74				/* Mega644 */
#define OWN_SIGNATURE		{ 0x1E, 0x96, 0x0A }		/* Mega644P */

/* SLA+R */
#define CMD_WAIT		0x00
#define CMD_READ_VERSION	0x01
#define CMD_READ_MEMORY		0x02

/* SLA+W */
#define CMD_SWITCH_APPLICATION	CMD_READ_VERSION
#define CMD_WRITE_MEMORY	CMD_READ_MEMORY

/* CMD_SWITCH_APPLICATION parameter */
#define BOOTTYPE_BOOTLOADER	0x00
#define BOOTTYPE_APPLICATION	0x80

/* CMD_{READ|WRITE}_* parameter */
#define MEMTYPE_CHIPINFO	0x00
#define MEMTYPE_FLASH		0x01
#define MEMTYPE_EEPROM		0x02
#define MEMTYPE_PARAMETERS	0x03


static void sendchar(uint8_t data)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = data;
}

static uint8_t recvchar(void)
{
	loop_until_bit_is_set(UCSR0A, RXC0);
	return UDR0;
}

static void print_byte(uint8_t val)
{
	uint8_t tmp = (val >> 4) & 0x0F;
	sendchar((tmp < 0x0A) ? ('0' + tmp) : ('a' - 0x0A + tmp));

	tmp = val & 0x0F;
	sendchar((tmp < 0x0A) ? ('0' + tmp) : ('a' - 0x0A + tmp));
}

static uint8_t i2c_master_tx(uint8_t val)
{
	TWDR = val;
	TWCR = (1<<TWINT) | (1<<TWEN);
	loop_until_bit_is_set(TWCR, TWINT);

	uint8_t status = TWSR & 0xF8;
	return status;
}

static uint8_t i2c_master_rx(uint8_t *val, uint8_t ack)
{
	if (ack)
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	else
		TWCR = (1<<TWINT) | (1<<TWEN);

	loop_until_bit_is_set(TWCR, TWINT);
	*val = TWDR;

	uint8_t status = TWSR & 0xF8;
	return status;
}

static void i2c_stop(void)
{
	PORTB |= LED_RT;
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}

static void i2c_start_address(uint8_t addr)
{
	while (1) {
		PORTB &= ~LED_RT;
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
		loop_until_bit_is_set(TWCR, TWINT);

		uint8_t status = i2c_master_tx(addr);
		if (status == 0x18 || status == 0x40)
			break;

		i2c_stop();
		_delay_ms(1);
	}
}

/* wait 100x 10ms */
volatile static uint8_t boot_timeout = 100;

ISR(TIMER0_OVF_vect)
{
	static uint8_t cnt;

	/* come back in 10ms (@20MHz) */
	TCNT0 = 0xFF - 195;

	/* blink LED while running */
	cnt++;
	cnt &= 0x03;
	if (cnt == 0x00)
		PORTB ^= LED_GN;

	if (boot_timeout > 0)
		boot_timeout--;
}

static uint16_t address;
static uint16_t page_size;
static uint8_t page_buf[SPM_PAGESIZE];


static void eraseFlash(void)
{
	uint32_t addr = 0;
	while (APP_END > addr) {
		boot_page_erase(addr);
		boot_spm_busy_wait();
		addr += SPM_PAGESIZE;
	}
	boot_rww_enable();
}

static uint16_t writeFlashPage(uint16_t waddr, uint16_t size)
{
	uint32_t pagestart = (uint32_t)waddr<<1;
	uint32_t baddr = pagestart;
	uint16_t data;
	uint8_t *tmp = page_buf;

	do {
		data = *tmp++;
		data |= *tmp++ << 8;
		boot_page_fill(baddr, data);

		baddr += 2;
		size -= 2;
	} while (size);

	boot_page_write(pagestart);
	boot_spm_busy_wait();
	boot_rww_enable();

	return baddr>>1;
}

static uint16_t writeEEpromPage(uint16_t address, uint16_t size)
{
	uint8_t *tmp = page_buf;

	do {
		EEARL = address;
		EEARH = (address >> 8);
		EEDR = *tmp++;
		address++;

		EECR |= (1<<EEMPE);
		EECR |= (1<<EEPE);
		eeprom_busy_wait();

		size--;
	} while (size);

	return address;
}

static uint16_t readFlashPage(uint16_t waddr, uint16_t size)
{
	uint32_t baddr = (uint32_t)waddr<<1;
	uint16_t data;

	do {
#if defined(RAMPZ)
		data = pgm_read_word_far(baddr);
#else
		data = pgm_read_word_near(baddr);
#endif
		sendchar(data);
		sendchar((data >> 8));
		baddr += 2;
		size -= 2;
	} while (size);

	return baddr>>1;
}

static uint16_t readEEpromPage(uint16_t address, uint16_t size)
{
	do {
		EEARL = address;
		EEARH = (address >> 8);
		EECR |= (1<<EERE);
		address++;

		sendchar(EEDR);

		size--;
	} while (size);

	return address;
}

/* 0-2: signature, 3: pagesize, 4-5: flash size, 6-7: eeprom size */
static uint8_t chipinfo[8];

void cmd_loop(void)
{
	uint8_t i2c_dev = 0;

	sendchar('F');
	sendchar('C');
	sendchar('_');
	sendchar('B');
	sendchar('O');
	sendchar('O');
	sendchar('T');

	while (1) {
		uint8_t val = recvchar();
		// Autoincrement?
		if (val == 'a') {
			sendchar('Y');

		// write address
        	} else if (val == 'A') {
			address = recvchar();
			address = (address << 8) | recvchar();
			sendchar('\r');

		// Buffer load support
		} else if (val == 'b') {
			sendchar('Y');
			page_size = (i2c_dev == 0) ? sizeof(page_buf) : chipinfo[3];
			sendchar((page_size >> 8) & 0xFF);
			sendchar(page_size & 0xFF);

		// Start buffer load
		} else if (val == 'B') {
			uint16_t size;
			uint16_t cnt;
			uint8_t *tmp = page_buf;

			size = recvchar() << 8;
			size |= recvchar();
			val = recvchar();

			for (cnt = 0; cnt < page_size; cnt++)
				*tmp++ = (cnt < size) ? recvchar() : 0xFF;

			if (i2c_dev != 0) {
				i2c_start_address(i2c_dev);
				i2c_master_tx(CMD_WRITE_MEMORY);
				i2c_master_tx((val == 'F') ? MEMTYPE_FLASH : MEMTYPE_EEPROM);
				i2c_master_tx(address >> 8);
				i2c_master_tx(address & 0xFF);

				address += page_size;

				tmp = page_buf;
				while (cnt--)
					i2c_master_tx(*tmp++);

				i2c_stop();

			} else {
				if (val == 'F') {
					address = writeFlashPage(address, size);
				} else if (val == 'E') {
					address = writeEEpromPage(address, size);
				}
			}
			sendchar('\r');

		// Block read
		} else if (val == 'g') {
			uint16_t size = recvchar() << 8;
			size |= recvchar();
			val = recvchar();

			if (i2c_dev != 0) {
				i2c_start_address(i2c_dev);
				i2c_master_tx(CMD_READ_MEMORY);
				i2c_master_tx((val == 'F') ? MEMTYPE_FLASH : MEMTYPE_EEPROM);
				i2c_master_tx(address >> 8);
				i2c_master_tx(address & 0xFF);

				i2c_start_address(i2c_dev | 0x01);
				while (size--) {
					i2c_master_rx(&val, size > 0);
					sendchar(val);
					address++;
				}

				i2c_stop();

			} else {
				if (val == 'F') {
					address = readFlashPage(address, size);
				} else if (val == 'E') {
					address = readEEpromPage(address, size);
				}
			}

		// Chip erase
		} else if (val == 'e') {
			eraseFlash();
			sendchar('\r');

		// Exit upgrade
		} else if (val == 'E') {
			boot_timeout = 0;
			sendchar('\r');
			_delay_ms(10);
			break;

		// Enter programming mode
		} else if (val == 'P') {
			if (i2c_dev != 0) {
				i2c_start_address(i2c_dev);
				i2c_master_tx(CMD_SWITCH_APPLICATION);
				i2c_master_tx(BOOTTYPE_BOOTLOADER);
				i2c_stop();

				_delay_ms(10);
			}
			sendchar('\r');

		// Leave programming mode
		} else if (val == 'L') {
			if (i2c_dev != 0) {
				i2c_start_address(i2c_dev);
				i2c_master_tx(CMD_SWITCH_APPLICATION);
				i2c_master_tx(BOOTTYPE_APPLICATION);
				i2c_stop();
			}
			sendchar('\r');

		// return programmer type
		} else if (val == 'p') {
			sendchar('S');

		// Return device type
		} else if (val == 't') {
			sendchar((i2c_dev == 0) ? OWN_DEVCODE : TWI_DEVCODE);
			sendchar(0);

		// clear and set LED ignored
        	} else if ((val == 'x') || (val == 'y')) {
			recvchar();
			sendchar('\r');

		// set device
		} else if (val == 'T') {
			recvchar();
			sendchar('\r');

		// Return software identifier
		} else if (val == 'S') {
			sendchar('F');
			sendchar('C');
			sendchar('_');
			sendchar('B');
			sendchar('O');
			sendchar('O');
			sendchar('T');

		// Return Software Version
		} else if (val == 'V' || val == 'v') {
			sendchar('0');
			sendchar('8');

		// Return Signature Bytes
		} else if (val == 's') {
			if (i2c_dev != 0) {
				sendchar(chipinfo[2]);
				sendchar(chipinfo[1]);
				sendchar(chipinfo[0]);

			} else {
				uint8_t sig[3] = OWN_SIGNATURE;
				sendchar(sig[2]);
				sendchar(sig[1]);
				sendchar(sig[0]);
			}

		// set i2c target
		} else if (val == '0') {
			i2c_dev = 0;
			sendchar(val);

		} else if (val >= '1' && val <= '4') {
			i2c_dev = (val - '0' + TWI_ADDRESS_BASE) << 1;
			sendchar(val);
			sendchar('<');

			i2c_start_address(i2c_dev);
			i2c_master_tx(CMD_SWITCH_APPLICATION);
			i2c_master_tx(BOOTTYPE_BOOTLOADER);
			i2c_stop();

			for (val = 0; val < 10; val++)
				_delay_ms(10);

			i2c_start_address(i2c_dev);
			i2c_master_tx(CMD_READ_VERSION);
			i2c_start_address(i2c_dev | 0x01);

			uint8_t cnt;
			for (cnt = 0; cnt < 16; cnt++) {
				i2c_master_rx(&val, (cnt != 15));
				sendchar(val);
			}

			i2c_stop();

			sendchar(',');

			i2c_start_address(i2c_dev);
			i2c_master_tx(CMD_READ_MEMORY);
			i2c_master_tx(MEMTYPE_CHIPINFO);
			i2c_master_tx(0x00);
			i2c_master_tx(0x00);
			i2c_start_address(i2c_dev | 0x01);

			uint8_t i;
			for (i = 0; i < sizeof(chipinfo); i++) {
				uint8_t more = (i < (sizeof(chipinfo) -1));
				i2c_master_rx(&chipinfo[i], more);

				sendchar('0');
				sendchar('x');
				print_byte(chipinfo[i]);
				sendchar(more ? ',' : '>');
			}
			i2c_stop();

		// test props
		} else if (val == 'k' || val == 'l') {
			if (i2c_dev != 0) {
				i2c_start_address(i2c_dev);
				i2c_master_tx(0x00 + (val - 'k') * 0x10);
				i2c_stop();

				sendchar(val);
			}

		// start Application / get Version
		} else if (val == 'I') {
			if (i2c_dev != 0) {
				i2c_start_address(i2c_dev);
				i2c_master_tx(CMD_SWITCH_APPLICATION);
				i2c_master_tx(BOOTTYPE_APPLICATION);
				i2c_stop();

				for (val = 0; val < 10; val++)
					_delay_ms(10);

				i2c_start_address(i2c_dev);
				i2c_master_tx(CMD_READ_VERSION);
				i2c_start_address(i2c_dev | 0x01);

				sendchar('<');
				uint8_t cnt;
				for (cnt = 0; cnt < 16; cnt++) {
					i2c_master_rx(&val, (cnt != 15));
					sendchar(val);
				}
				sendchar('>');
				i2c_stop();
			}

		/* ESC */
		} else if (val != 0x1b) {
			sendchar('?');
		}
	}
}

static void (*jump_to_app)(void) __attribute__ ((noreturn)) = 0x0000;

/*
 * For newer devices the watchdog timer remains active even after a
 * system reset. So disable it as soon as possible.
 * automagically called on startup
 */
#if defined (__AVR_ATmega88__) || defined (__AVR_ATmega168__)
void disable_wdt_timer(void) __attribute__((naked, section(".init3")));
void disable_wdt_timer(void)
{
	MCUSR = 0;
	WDTCSR = (1<<WDCE) | (1<<WDE);
	WDTCSR = (0<<WDE);
}
#endif

int main(void) __attribute__ ((noreturn));
int main(void)
{
	DDRB = LED_GN | LED_RT;
	PORTB = LED_GN | LED_RT;

	/* move interrupt-vectors to bootloader */
	MCUCR = (1<<IVCE);
	MCUCR = (1<<IVSEL);

	/* Set baudrate */
	UBRR0H = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
	UBRR0L = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

	/* USART: rx/tx enable, 8n1 */
	UCSR0B = (1<<TXEN0) | (1<<RXEN0);
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);

	/* timer0: running with F_CPU/1024 */
	TCCR0B = (1<<CS02) | (1<<CS00);

	/* enable timer0 OVF interrupt */
	TIMSK0 |= (1<<TOIE0);

	/* enable TWI interface */
	TWBR = ((F_CPU/TWI_CLK)-16)/2;
	TWCR = (1<<TWSTO) | (1<<TWEN);

	sei();

	while (boot_timeout > 0) {
		if (UCSR0A & (1<<RXC0))
			if (recvchar() == 'S')
				cmd_loop();
	}

	cli();

	/* move interrupt-vectors to applicaion */
	MCUCR = (1<<IVCE);
	MCUCR = (0<<IVSEL);

	PORTB = LED_RT;

	uint8_t i;
	for (i = 0; i < 10; i++)
		_delay_ms(10);

	jump_to_app();
}
