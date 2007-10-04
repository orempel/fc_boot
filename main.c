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

#define TWI_CLK			200000

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

static uint8_t i2c_cmd;
static uint8_t i2c_dev;
static uint16_t i2c_address;
static uint8_t i2c_buf[64];
static volatile uint8_t i2c_complete;

void i2c_transfer(uint8_t cmd)
{
	i2c_complete = 0;
	i2c_cmd = cmd;
	TWCR |= (1<<TWINT) | (1<<TWSTA);

	PORTB |= LED_RT;
	while (i2c_complete != 1) {
		if (i2c_complete == 2) {
			_delay_ms(5);
			TWCR |= (1<<TWINT) | (1<<TWSTA);
		}
	}
	PORTB &= ~LED_RT;
}

ISR(TWI_vect)
{
	static uint8_t bcnt;

	uint8_t status = TWSR & 0xF8;
// 	printf("status: 0x%02x 0x%02x 0x%02x\n", i2c_cmd, status, bcnt);

	switch (status) {
	/* START transmitted -> send SLA+W */
	case 0x08:
		bcnt = 0;
		TWDR = (i2c_dev << 1);
		TWCR &= ~(1<<TWSTA);
		TWCR |= (1<<TWINT);
		break;

	/* repeated START transmitted -> send SLA+R */
	case 0x10:
		bcnt = 0;
		TWDR = (i2c_dev << 1) | 0x01;
		TWCR &= ~(1<<TWSTA);
		TWCR |= (1<<TWINT);
		break;

	/* SLA+W transmitted, ACK received -> send command */
	case 0x18:
		bcnt++;
		TWDR = i2c_cmd;
		TWCR |= (1<<TWINT);
		break;

	/* data transmitted, ACK received -> ? */
	case 0x28:
		switch (i2c_cmd) {
		case CMD_GET_INFO:
		case CMD_GET_SIGNATURE:
		case CMD_READ_FLASH:
		case CMD_READ_EEPROM:
			TWCR |= (1<<TWINT) | (1<<TWSTA);
			break;

		case CMD_WRITE_FLASH:
		case CMD_WRITE_EEPROM:
			if (bcnt == 1) {
				TWDR = (i2c_address >> 8);

			} else if (bcnt == 2) {
				TWDR = (i2c_address & 0xFF);

			} else if (bcnt == 3) {
				TWDR = (WRITE_COOKIE >> 8);

			} else if (bcnt == 4) {
				TWDR = (WRITE_COOKIE & 0xFF);

			} else if (bcnt >= 5 && bcnt < (64 +5)) {
				TWDR = i2c_buf[bcnt -5];

			} else {
				i2c_complete = 1;
				TWCR |= (1<<TWSTO);
			}
			TWCR |= (1<<TWINT);
			break;

		default:
		case CMD_BOOT_APPLICATION:
		case CMD_BOOT_LOADER:
			i2c_complete = 1;
			TWCR |= (1<<TWINT) | (1<<TWSTO);
			break;
		}
		bcnt++;
		break;

	/* SLA+R transmitted, ACK received -> send ACK (send more data) */
	case 0x40:
		TWCR |= (1<<TWINT) | (1<<TWEA);
		break;

	/* data received, ACK sent */
	case 0x50:
		i2c_buf[bcnt++] = TWDR;

		if ((i2c_cmd == CMD_GET_INFO && bcnt < 16) ||
		    (i2c_cmd == CMD_GET_SIGNATURE && bcnt < 4) ||
		    ((i2c_cmd == CMD_READ_FLASH || i2c_cmd == CMD_READ_EEPROM) && bcnt < 64)) {
			TWCR |= (1<<TWEA);
		} else {
			TWCR &= ~(1<<TWEA);
		}
		TWCR |= (1<<TWINT);
		break;

	/* data received, NACK sent */
	case 0x58:
		i2c_buf[bcnt++] = TWDR;
		i2c_complete = 1;
		TWCR |= (1<<TWINT) | (1<<TWSTO);
		break;

	/* SLA+W transmitted, NACK received */
	case 0x20:
	/* data transmitted, NACK received */
	case 0x30:
	/* arbitation lost during SLA+W, SLA+R or data */
	case 0x38:
	/* SLA+R transmitted, NACK received */
	default:
		i2c_complete = 2;
		TWCR |= (1<<TWINT) | (1<<TWSTO);
		break;
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
	uint8_t dev = 0;

	i2c_dev = 0x21;

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

			if (dev == DEVCODE_M8) {
				if (val == 'F') {
					tmp = page_buf;
					while (size) {
						i2c_address = address;
						for (cnt = 0; cnt < sizeof(i2c_buf); cnt++)
							i2c_buf[cnt] = *tmp++;

						i2c_transfer(CMD_WRITE_FLASH);
						address += sizeof(i2c_buf);

						size -= sizeof(i2c_buf);
					}

				} else if (val == 'E') {

				}

			} else if (dev == DEVCODE_M644) {
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

			if (dev == DEVCODE_M8) {
				if (val == 'F') {
					while (size > 0) {
						uint8_t i = 0;
						i2c_address = address;
						i2c_transfer(CMD_READ_FLASH);

						address += sizeof(i2c_buf);
						while (size-- && i < sizeof(i2c_buf))
							sendchar(i2c_buf[i++]);
					}

				} else if (val == 'E') {
					sendchar(0xff);

				}

			} else if (dev == DEVCODE_M644) {
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
			/* boot loader */
			i2c_transfer(CMD_BOOT_LOADER);
			sendchar('\r');

		// Leave programming mode
		} else if (val == 'L') {
			/* boot application */
			i2c_transfer(CMD_BOOT_APPLICATION);
			sendchar('\r');

		// return programmer type
		} else if (val == 'p') {
			sendchar('S');

		// Return device type
		} else if (val == 't') {
			sendchar(DEVCODE_M8);
			sendchar(DEVCODE_M644);
			sendchar(0);

		// clear and set LED ignored
        	} else if ((val == 'x') || (val == 'y')) {
			recvchar();
			sendchar('\r');

		// set device
		} else if (val == 'T') {
			dev = recvchar();
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
		} else if (val == 'V') {
			sendchar('0');
			sendchar('8');

		// Return Signature Bytes
		} else if (val == 's') {
			if (dev == DEVCODE_M8) {
				sendchar(0x07);
				sendchar(0x93);
				sendchar(0x1e);

			} else if (dev == DEVCODE_M644) {
				sendchar(0x09);
				sendchar(0x96);
				sendchar(0x1e);

			} else {
				sendchar(0xFF);
				sendchar(0xFF);
				sendchar(0xFF);
			}

		// set i2c target
		} else if (val >= '1' && val <= '4') {
			i2c_dev = val - '1' + 0x21;
			sendchar(val);

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
	TWCR = (1<<TWSTO) | (1<<TWEN) | (1<<TWIE);

	sei();

	while (boot_timeout > 0) {
		if (UCSR0A & (1<<RXC0))
			cmd_loop();
	}

	cli();

	/* disable TWI */
	TWCR = 0x00;

	/* disable Timer0 */
	TIMSK0 = 0x00;
	TCCR0B = 0x00;

	/* disable USART */
	UCSR0C = 0x00;
	UCSR0B = 0x00;

	/* move interrupt-vectors to applicaion */
	MCUCR = (1<<IVCE);
	MCUCR = (0<<IVSEL);

	PORTB = 0x00;

	uint8_t i;
	for (i = 0; i < 10; i++)
		_delay_ms(10);

	jump_to_app();

	return 0;
}
