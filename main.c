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
 * Fuse H: 0xDC (1024 words bootloader)
 * Fuse L: 0xFF (ext. Crystal)
 */

#define F_CPU 20000000
#include <util/delay.h>

#define BAUDRATE 57600
#define UART_CALC_BAUDRATE(baudRate) ((uint32_t)(F_CPU) / ((uint32_t)(baudRate)*16) -1)

#define APP_END			0xF7FF

#define LED_RT			(1<<PORTB0)			/* low active */
#define LED_GN			(1<<PORTB1)			/* high active */

#define TWI_CLK			100000
#define TWI_ADDRESS_BASE	0x28				/* 0x29 - 0x2C */

#define TWI_DEVCODE		0x76				/* Mega8 */
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

static void i2c_master_tx(uint8_t val)
{
	TWDR = val;
	TWCR = (1<<TWINT) | (1<<TWEN);
	loop_until_bit_is_set(TWCR, TWINT);
}

static uint8_t i2c_master_rx(uint8_t ack)
{
	if (ack)
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	else
		TWCR = (1<<TWINT) | (1<<TWEN);

	loop_until_bit_is_set(TWCR, TWINT);
	return TWDR;
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

		i2c_master_tx(addr);
		uint8_t status = (TWSR & 0xF8);
		if (status == 0x18 || status == 0x40)
			break;

		i2c_stop();
		_delay_ms(1);
	}
}

static void i2c_switch_app(uint8_t i2c_dev, uint8_t app)
{
	i2c_start_address(i2c_dev);
	i2c_master_tx(CMD_SWITCH_APPLICATION);
	i2c_master_tx(app);
	i2c_stop();

	_delay_ms(20);
}

static void i2c_print_version(uint8_t i2c_dev)
{
	i2c_start_address(i2c_dev);
	i2c_master_tx(CMD_READ_VERSION);
	i2c_start_address(i2c_dev | 0x01);

	uint8_t cnt = 16;
	while (cnt--) {
		sendchar(i2c_master_rx(cnt));
	}

	i2c_stop();
}

static uint8_t page_buf[SPM_PAGESIZE];

static void eraseFlash(void)
{
	uint16_t address = 0;
	while (APP_END < address) {
		boot_page_erase(address);
		boot_spm_busy_wait();
		address += SPM_PAGESIZE;
	}
	boot_rww_enable();
}

static void writeFlashPage(uint16_t address, uint16_t size)
{
	uint16_t pagestart = address;
	uint16_t data;
	uint8_t *tmp = page_buf;

	do {
		data = *tmp++;
		data |= *tmp++ << 8;
		boot_page_fill(address, data);

		address += 2;
		size -= 2;
	} while (size);

	boot_page_write(pagestart);
	boot_spm_busy_wait();
	boot_rww_enable();
}

static void writeEEpromPage(uint16_t address, uint16_t size)
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
}

static void readFlashPage(uint16_t address, uint16_t size)
{
	uint16_t data;

	do {
		data = pgm_read_word_near(address);
		sendchar(data);
		sendchar((data >> 8));
		address += 2;
		size -= 2;
	} while (size);
}

static void readEEpromPage(uint16_t address, uint16_t size)
{
	do {
		EEARL = address;
		EEARH = (address >> 8);
		EECR |= (1<<EERE);
		address++;

		sendchar(EEDR);

		size--;
	} while (size);
}

/* 0-2: signature, 3: pagesize, 4-5: flash size, 6-7: eeprom size */
static uint8_t chipinfo[8];

void cmd_loop(uint8_t val)
{
	uint16_t address = 0;
	uint16_t page_size = 0;
	uint8_t i2c_dev = 0;

	while (1) {
		uint8_t response = 0xFF;

		// Autoincrement?
		if (val == 'a') {
			response = 'Y';

		// write address
        	} else if (val == 'A') {
			address = recvchar();
			address = (address << 8) | recvchar();
			response = '\r';

		// Buffer load support
		} else if (val == 'b') {
			sendchar('Y');
			page_size = (i2c_dev == 0) ? sizeof(page_buf) : chipinfo[3];
			sendchar((page_size >> 8) & 0xFF);
			response = page_size & 0Xff;

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
					writeFlashPage(address, size);
				} else if (val == 'E') {
					writeEEpromPage(address, size);
				}
				address += size;
			}
			response = '\r';

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
					sendchar(i2c_master_rx(size > 0));
					address++;
				}

				i2c_stop();

			} else {
				if (val == 'F') {
					readFlashPage(address, size);
				} else if (val == 'E') {
					readEEpromPage(address, size);
				}
				address += size;
			}

		// Chip erase
		} else if (val == 'e') {
			if (i2c_dev == 0) {
				eraseFlash();
			}
			response = '\r';

		// Exit upgrade
		} else if (val == 'E') {
			sendchar('\r');
			if (i2c_dev == 0) {
				return;
			}

		// Enter / Leave programming mode
		} else if (val == 'P' || val == 'L') {
			if (i2c_dev != 0) {
				val = (val == 'P') ? BOOTTYPE_BOOTLOADER : BOOTTYPE_APPLICATION;
				i2c_switch_app(i2c_dev, val);
			}
			response = '\r';

		// return programmer type
		} else if (val == 'p') {
			response = 'S';

		// Return device type
		} else if (val == 't') {
			sendchar((i2c_dev == 0) ? OWN_DEVCODE : TWI_DEVCODE);
			response = '\0';

		// clear and set LED ignored
		} else if ((val == 'x') || (val == 'y') || (val == 'T')) {
			recvchar();
			response = '\r';

		// Return software identifier
		} else if (val == 'S') {
			sendchar('F');
			sendchar('C');
			sendchar('_');
			sendchar('B');
			sendchar('O');
			sendchar('O');
			response = 'T';

		// Return Software Version
		} else if (val == 'V' || val == 'v') {
			sendchar('0');
			response = '8';

		// Return Signature Bytes
		} else if (val == 's') {
			if (i2c_dev != 0) {
				sendchar(chipinfo[2]);
				sendchar(chipinfo[1]);
				response = chipinfo[0];

			} else {
				uint8_t sig[3] = OWN_SIGNATURE;
				sendchar(sig[2]);
				sendchar(sig[1]);
				response = sig[0];
			}

		// set i2c target
		} else if (val == '0') {
			i2c_dev = 0;

		} else if (val >= '1' && val <= '4') {
			i2c_dev = (val - '0' + TWI_ADDRESS_BASE) << 1;

			i2c_switch_app(i2c_dev, BOOTTYPE_BOOTLOADER);
			_delay_ms(100);
			i2c_print_version(i2c_dev);

			i2c_start_address(i2c_dev);
			i2c_master_tx(CMD_READ_MEMORY);
			i2c_master_tx(MEMTYPE_CHIPINFO);
			i2c_master_tx(0x00);
			i2c_master_tx(0x00);
			i2c_start_address(i2c_dev | 0x01);

			uint8_t i;
			for (i = 0; i < sizeof(chipinfo); i++) {
				uint8_t more = (i < (sizeof(chipinfo) -1));
				chipinfo[i] = i2c_master_rx(more);
			}
			i2c_stop();

		// test props
		} else if (val == 'k' || val == 'l') {
			if (i2c_dev != 0) {
				i2c_start_address(i2c_dev);
				i2c_master_tx(0x00 + (val - 'k') * 0x10);
				i2c_stop();
				response = val;
			}

		// get Version
		} else if (val == 'I') {
			if (i2c_dev != 0) {
				i2c_switch_app(i2c_dev, BOOTTYPE_APPLICATION);
				_delay_ms(100);
				i2c_print_version(i2c_dev);
			}

		// fake MK-TOOL specific stuff
		} else if (val == 0xAA) {
			sendchar('M');
			sendchar('K');
			sendchar('B');
			response = 'L';

		/* ESC */
		} else if (val != 0x1b) {
			response = '?';
		}

		if (response != 0xFF) {
			sendchar(response);
		}

		val = recvchar();
	}
}

static void (*jump_to_app)(void) __attribute__ ((noreturn)) = 0x0000;

/*
 * For newer devices the watchdog timer remains active even after a
 * system reset. So disable it as soon as possible.
 * automagically called on startup
 */
void disable_wdt_timer(void) __attribute__((naked, section(".init3")));
void disable_wdt_timer(void)
{
	MCUSR = 0;
	WDTCSR = (1<<WDCE) | (1<<WDE);
	WDTCSR = (0<<WDE);
}

/* linking without vectors, still need __vector_default */
void __vector_default(void) {}

int main(void) __attribute__ ((noreturn));
int main(void)
{
	DDRB = LED_GN | LED_RT;
	PORTB = LED_RT;

	/* Set baudrate */
	UBRR0H = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
	UBRR0L = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

	/* USART: rx/tx enable, 8n1 */
	UCSR0B = (1<<TXEN0) | (1<<RXEN0);
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);

	/* enable TWI interface */
	TWBR = ((F_CPU/TWI_CLK)-16)/2;
	TWCR = (1<<TWSTO) | (1<<TWEN);

	uint8_t prev = 0x00;
	uint8_t boot_timeout = 100;

	while (boot_timeout-- > 0) {
		if (UCSR0A & (1<<RXC0)) {
			uint8_t val = recvchar();

			if (prev == 0x1B && (val == 0xAA || val == 'S')) {
				PORTB |= LED_GN;
				cmd_loop(val);
				boot_timeout = 0;
			}

			prev = val;
		}
		_delay_ms(10);

		if (!(boot_timeout & 0x03))
			PORTB ^= LED_GN;
	}

	PORTB = LED_RT;

	jump_to_app();
}
