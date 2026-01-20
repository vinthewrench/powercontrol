# ---------------- Configuration ----------------
MCU     = atmega88pb

TARGET  = powercontrol
F_CPU   = 8000000UL

CC      = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude

CFLAGS  = -Wall -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU)
LDFLAGS = -mmcu=$(MCU)

# ---------------- Default build ----------------
all: $(TARGET).hex

$(TARGET).elf: $(TARGET).c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

flash: $(TARGET).hex
	$(AVRDUDE) -c atmelice_isp -p m88pb -U flash:w:$<

# ---------------- Fuse utilities ----------------
checkfuses:
	$(AVRDUDE) -c atmelice_isp -p m88pb \
		-U lfuse:r:-:h \
		-U hfuse:r:-:h \
		-U efuse:r:-:h

# Default ATmega8 internal 8 MHz RC (CKDIV8 disabled)
setfuses:
	$(AVRDUDE) -c atmelice_isp -p m88pb \
		-U lfuse:w:0xE2:m \
		-U hfuse:w:0xDF:m \
		-U efuse:w:0xFF:m

# ---------------- Cleanup ----------------
clean:
	rm -f *.o *.elf *.hex
