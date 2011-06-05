PRG            = fc_boot
OBJ            = main.o
MCU_TARGET     = atmega644p
OPTIMIZE       = -Os

DEFS           =
LIBS           =

LDSCRIPT_SRC  := /usr/lib/ldscripts/avr5.x
LDSCRIPT_DST  := ldscript-no-vectors.x

# Override is only needed by avr-lib build system.
override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map,--section-start=.text=0xF800,-T$(LDSCRIPT_DST)

CC             = avr-gcc
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
SIZE           = avr-size

#LDSCRIPT_SRC  := $(shell LANG=C $(CC) $(CFLAGS) -Wl,--verbose 2> /dev/null | awk '/^opened script file (.*)$$/{ print $$4 }')

all: $(PRG).elf lst text
	$(SIZE) -x -A $(PRG).elf

$(PRG).elf: $(OBJ) | $(LDSCRIPT_DST)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

$(LDSCRIPT_DST): $(LDSCRIPT_SRC)
# remove all lines with *vectors* and insert DISCARD line above .text declaration
	sed -e '/.*vectors.*/d' -e 's/\(^[ \t]*\)\(.text[ \t]*\:\)$$/\1\/DISCARD\/ : { *(.vectors) }\n\1\2/g' $^ > $@
#	-diff -uNr $^ $@ > $@.diff

clean:
	rm -rf *.o $(PRG).lst $(PRG).map $(PRG).elf $(PRG).hex $(PRG).bin $(LDSCRIPT_DST)

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
	avrdude -c dragon_isp -P usb -p m644p -V -U flash:w:$(PRG).hex
