AVR_MCU ?= attiny84a
AVR_F_CPU ?= 8000000UL

AVR_LFUSE ?= 0xE2
AVR_HFUSE ?= 0xDF

FIRMWARE_CFLAGS = \
	-DI2C_REGISTER_ALLOC=1 \
	-Wno-implicit-fallthrough \
	$(NULL)

FIRMWARE_SOURCES = \
	test.c \
	i2c-target.c \
	$(NULL)

FIRMWARE_HEADERS = \
	i2c-target.h \
	$(NULL)

build-test:
	set -e; \
	for i in {2,4,8}4{,a} {2,4,8}5 2313{,a} 4313; do \
		$(MAKE) clean all AVR_MCU=attiny$$i; \
	done
	$(MAKE) clean

.PHONY: build-test
