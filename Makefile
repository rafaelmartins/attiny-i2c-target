# programmer and mcu settings
AVRDUDE_PROGRAMMER ?= avrispmkII
AVRDUDE_PORT ?= usb
AVR_MCU ?= attiny84
AVR_CPU_FREQUENCY ?= 8000000UL

AVRDUDE = avrdude
CC = avr-gcc
OBJCOPY = avr-objcopy
SIZE = avr-size

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
		-U flash:w:$<

clean:
	-$(RM) \
		firmware.elf \
		firmware.hex

.PHONY: all size flash clean