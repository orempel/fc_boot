CC	:= avr-gcc
LD	:= avr-ld
OBJCOPY	:= avr-objcopy
OBJDUMP	:= avr-objdump
SIZE	:= avr-size

TARGET = fc_boot
SOURCE = $(wildcard *.c)

# select MCU
MCU = atmega644p

AVRDUDE_PROG := -c avr910 -b 115200 -P /dev/ttyUSB0
#AVRDUDE_PROG := -c dragon_isp -P usb

# ---------------------------------------------------------------------------

ifeq ($(MCU), atmega644p)
AVRDUDE_MCU=m644p

# (20MHz ext. Crystal, 2.7V BOD)
AVRDUDE_FUSES=lfuse:w:0xff:m hfuse:w:0xdc:m efuse:w:0xfd:m
BOOTLOADER_START=0xF800
endif

# ---------------------------------------------------------------------------

CFLAGS = -pipe -g -Os -mmcu=$(MCU) -Wall
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -Wa,-adhlns=$(*F).lst
CFLAGS += -DBOOTLOADER_START=$(BOOTLOADER_START)
LDFLAGS = -Wl,-Map,$(@:.elf=.map),--cref,--section-start=.text=$(BOOTLOADER_START)
LDFLAGS += -Wl,--relax,--gc-sections

LDSCRIPT := $(shell LANG=C $(CC) $(CFLAGS) -Wl,--verbose 2> /dev/null | awk '/^opened script file (.*)$$/{ print $$4 }')
LDSCRIPT_NOVECT := ldscript-no-vectors-$(notdir $(LDSCRIPT))

# ---------------------------------------------------------------------------

$(TARGET): $(TARGET).elf
	@$(SIZE) -B -x --mcu=$(MCU) $<

$(TARGET).elf: $(SOURCE:.c=.o) | $(LDSCRIPT_NOVECT)
	@echo " Linking file:  $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -Wl,-T$(LDSCRIPT_NOVECT) -o $@ $^ 2> /dev/null
	@$(OBJDUMP) -h -S $@ > $(@:.elf=.lss)
	@$(OBJCOPY) -j .text -j .data -O ihex $@ $(@:.elf=.hex)
	@$(OBJCOPY) -j .text -j .data -O binary $@ $(@:.elf=.bin)

%.o: %.c $(MAKEFILE_LIST)
	@echo " Building file: $<"
	@$(CC) $(CFLAGS) -o $@ -c $<

# remove interrupt vector section from avr-libc linker script
# (remove all lines with *vectors* and insert DISCARD line above .text declaration)
$(LDSCRIPT_NOVECT): $(LDSCRIPT) $(MAKEFILE_LIST)
	@echo " Creating:      $@"
	@sed -e '/.*vectors.*/d' -e 's/\(^[ \t]*\)\(.text[ \t]*\:\)$$/\1\/DISCARD\/ : { *(.vectors) }\n\1\2/g' $< > $@
#	-@diff -uNr $< $@ > $@.diff

clean:
	rm -rf $(SOURCE:.c=.o) $(SOURCE:.c=.lst) $(addprefix $(TARGET), .elf .map .lss .hex .bin) $(LDSCRIPT_NOVECT)

install: $(TARGET).elf
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -U flash:w:$(<:.elf=.hex)

fuses:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) $(patsubst %,-U %, $(AVRDUDE_FUSES))
