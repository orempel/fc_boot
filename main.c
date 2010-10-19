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

#include <stdio.h>

#define F_CPU 20000000
#include <util/delay.h>

#define BAUDRATE 19200
#define UART_CALC_BAUDRATE(baudRate) ((uint32_t)(F_CPU) / ((uint32_t)(baudRate)*16) -1)

#define APP_END		0xF000

#define LED_RT			(1<<PORTB0)
#define LED_GN			(1<<PORTB1)

#define TWI_CLK			100000

#define WRITE_COOKIE		0x4711
#define CMD_WAIT		0x00
#define CMD_GET_INFO		0x10
#define CMD_GET_SIGNATURE	0x11
#define CMD_WRITE_FLASH		0x12
#define CMD_READ_FLASH		0x13
#define CMD_WRITE_EEPROM	0x14
#define CMD_READ_EEPROM		0x15
#define CMD_BOOT_APPLICATION	0x1F

#define CMD_SET_PWM		0x21
#define CMD_GET_STATUS		0x22
// #define CMD_SET_PARAM		0x23
// #define CMD_GET_PARAM		0x24
#define CMD_BOOT_LOADER		0x2F

#define DEVCODE_M8	0x77
#define DEVCODE_M644	0x74

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

static int uart_putchar(char c, FILE *stream)
{
	if (c == '\n')
		uart_putchar('\r', stream);
	sendchar(c);
	return 0;
}

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

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
	PORTB &= ~LED_RT;
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}

static void i2c_start_address(uint8_t addr)
{
	while (1) {
		PORTB |= LED_RT;
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
static uint8_t boot_timeout = 100;

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
static uint8_t page_buf[256];

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
			address = (address<<8) | recvchar();
			sendchar('\r');

		// Buffer load support
		} else if (val == 'b') {
			sendchar('Y');
			sendchar((sizeof(page_buf) >> 8) & 0xFF);
			sendchar(sizeof(page_buf) & 0xFF);

		// Start buffer load
		} else if (val == 'B') {
			uint16_t size;
			uint16_t cnt;
			uint8_t *tmp = page_buf;

			size = recvchar() << 8;
			size |= recvchar();
			val = recvchar();

			for (cnt = 0; cnt < sizeof(page_buf); cnt++)
				*tmp++ = (cnt < size) ? recvchar() : 0xFF;

			if (i2c_dev != 0) {
				i2c_start_address(i2c_dev);
				if (val == 'F') {
					i2c_master_tx(CMD_WRITE_FLASH);
					// FIXME: Flashwriting is not working. see bl_master (which works)

				} else {
					i2c_master_tx(CMD_WRITE_EEPROM);
					/* no page align, transfer only needed bytes */
					cnt = size;
				}

				i2c_master_tx(address >> 8);
				i2c_master_tx(address & 0xFF);
				address += sizeof(page_buf);

				i2c_master_tx(WRITE_COOKIE >> 8);
				i2c_master_tx(WRITE_COOKIE & 0xFF);

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
				if (val == 'F')
					i2c_master_tx(CMD_READ_FLASH);
				else
					i2c_master_tx(CMD_READ_EEPROM);

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
				i2c_master_tx(CMD_BOOT_LOADER);
				i2c_stop();

				_delay_ms(10);

				i2c_start_address(i2c_dev);
				i2c_master_tx(CMD_GET_SIGNATURE);
				i2c_start_address(i2c_dev | 0x01);
				i2c_master_rx(&val, 1);
				i2c_master_rx(&val, 1);
				i2c_master_rx(&val, 0);
				i2c_stop();
			}
			sendchar('\r');

		// Leave programming mode
		} else if (val == 'L') {
			if (i2c_dev != 0) {
				i2c_start_address(i2c_dev);
				i2c_master_tx(CMD_BOOT_APPLICATION);
				i2c_stop();
			}
			sendchar('\r');

		// return programmer type
		} else if (val == 'p') {
			sendchar('S');

		// Return device type
		} else if (val == 't') {
			sendchar((i2c_dev == 0) ? DEVCODE_M644 : DEVCODE_M8);
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
				sendchar(0x07);
				sendchar(0x93);
				sendchar(0x1e);

			} else {
				sendchar(0x09);
				sendchar(0x96);
				sendchar(0x1e);
			}

		// set i2c target
		} else if (val == '0') {
			i2c_dev = 0;
			sendchar(val);

		} else if (val >= '1' && val <= '4') {
			i2c_dev = (val - '1' + 0x21) << 1;
			sendchar(val);

		// test props
		} else if (val == 'l') {
			i2c_start_address(i2c_dev);
			i2c_master_tx(CMD_SET_PWM);
			i2c_master_tx(0x00);
			i2c_stop();

		// test props
		} else if (val == 'k') {
			i2c_start_address(i2c_dev);
			i2c_master_tx(CMD_SET_PWM);
			i2c_master_tx(0x0f);
			i2c_stop();

		// get Info
		} else if (val == 'I') {
			i2c_start_address(i2c_dev);
			i2c_master_tx(CMD_GET_INFO);
			i2c_start_address(i2c_dev | 0x01);

			uint8_t cnt;
			for (cnt = 0; cnt < 16; cnt++) {
				i2c_master_rx(&val, (cnt != 15));
				sendchar(val);
			}

			i2c_stop();

		/* ESC */
		} else if (val != 0x1b) {
			sendchar('?');
		}
	}
}

static void (*jump_to_app)(void) = 0x0000;

int main(void)
{
	DDRB = LED_GN | LED_RT;
	PORTB = LED_GN;

	/* move interrupt-vectors to bootloader */
	MCUCR = (1<<IVCE);
	MCUCR = (1<<IVSEL);

	/* Set baudrate */
	UBRR0H = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
	UBRR0L = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

	/* USART: rx/tx enable, 8n1 */
	UCSR0B = (1<<TXEN0) | (1<<RXEN0);
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);

	stdout = &mystdout;

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

	jump_to_app();

	return 0;
}
