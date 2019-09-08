# programmer and mcu settings
AVRDUDE_PROGRAMMER ?= avrispmkII
AVRDUDE_PORT ?= usb
AVR_MCU ?= attiny84a
AVR_CPU_FREQUENCY ?= 8000000UL

AVRDUDE = avrdude
CC = avr-gcc
OBJCOPY = avr-objcopy
SIZE = avr-size

# disable clock/8 (run at 8mhz)
AVR_FUSE_LOW = 0xE2

# spi programming enabled
AVR_FUSE_HIGH_SPIEN = 0xDF
# debugwire enabled
AVR_FUSE_HIGH_DWEN = 0x9F

AVR_FUSE_EXTENDED = 0xFF
AVR_LOCKBIT = 0xFF

CFLAGS = \
	-std=gnu99 \
	-mmcu=$(AVR_MCU) \
	-DF_CPU=$(AVR_CPU_FREQUENCY) \
	-DI2C_REGISTER_ALLOC=1 \
	-Os \
	-funsigned-char \
	-funsigned-bitfields \
	-fpack-struct \
	-fshort-enums \
	-fno-unit-at-a-time \
	-Wall \
	-Wno-implicit-fallthrough \
	-Wextra \
	-Werror \
	$(NULL)

SOURCES = \
	test.c \
	i2c-device.c \
	$(NULL)

HEADERS = \
	i2c-device.h \
	$(NULL)

all: firmware.hex

%.hex: %.elf
	$(OBJCOPY) \
		-O ihex \
		-j .data \
		-j .text \
		$< \
		$@

%.elf: $(SOURCES) $(HEADERS) Makefile
	$(CC) \
		$(CFLAGS) \
		$(SOURCES) \
		-o $@
	@$(MAKE) --no-print-directory size

size: firmware.elf
	@echo;$(SIZE) \
		--mcu=$(AVR_MCU) \
		-C $<

flash: firmware.hex
	$(AVRDUDE) \
		-p $(AVR_MCU) \
		-c $(AVRDUDE_PROGRAMMER) \
		-P $(AVRDUDE_PORT) \
		-U flash:w:$< \
		-U lfuse:w:$(AVR_FUSE_LOW):m \
		-U hfuse:w:$(AVR_FUSE_HIGH_SPIEN):m \
		-U efuse:w:$(AVR_FUSE_EXTENDED):m \
		-U lock:w:$(AVR_LOCKBIT):m

dw:
	$(AVRDUDE) \
		-p $(AVR_MCU) \
		-c $(AVRDUDE_PROGRAMMER) \
		-P $(AVRDUDE_PORT) \
		-U hfuse:w:$(AVR_FUSE_HIGH_DWEN):m

clean:
	-$(RM) \
		firmware.elf \
		firmware.hex

build-test:
	set -e; \
	for i in {2,4,8}4{,a} {2,4,8}5 2313{,a} 4313; do \
		$(MAKE) clean all AVR_MCU=attiny$$i; \
	done

.PHONY: all size flash dw clean build-test
