PRG            = fc_boot
OBJ            = main.o
MCU_TARGET     = atmega644
OPTIMIZE       = -Os

DEFS           =
LIBS           =

# Override is only needed by avr-lib build system.
override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map,--section-start=.text=0xF000

CC             = avr-gcc
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
SIZE           = avr-size

all: $(PRG).elf lst text
	$(SIZE) -x -A $(PRG).elf

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o *.lst *.map $(PRG).elf *.hex *.bin

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

text: hex bin

hex:  $(PRG).hex
bin:  $(PRG).bin

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

install: text
#	uisp -dprog=avr910 -dserial=/dev/ttyS0 -dspeed=115200 -dpart=M8 --erase --upload if=$(PRG).hex
	avrdude -p m644 -c butterfly -b 115200 -P /dev/ttyS0 -u -V -U flash:w:$(PRG).hex