
BINARY = $(notdir $(shell pwd))

SRCDIR = .
BUILD_DIR = bin
INCLUDE = -I../include -I../../include -I../../../include

ARCH_FLAGS = -mmcu=atmega1284p

CC = avr-gcc
LD = avr-gcc
OBJCOPY = avr-objcopy

SOURCES = $(wildcard *.c)
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES:.c=.o)))

CFLAGS += -Os -g
#CFLAGS += -Wextra -Wshadow -Wimplicit-function-declaration
#CFLAGS += -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes

LDFLAGS += -g

OBJFLAGS += -j .text -j .data -O ihex

all: elf hex

elf: $(BINARY).elf
hex: elf $(BINARY).hex

flash:
	avrdude -F -c usbasp -p m1284p -P usb -U flash:w:$(BUILD_DIR)/$(BINARY).hex

%.elf: $(OBJECTS)
	@mkdir -p $(BUILD_DIR)
	$(LD) $(LDFLAGS) $(ARCH_FLAGS) -o $(BUILD_DIR)/$@ $(OBJECTS)

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(INCLUDE) $(CFLAGS) $(ARCH_FLAGS) -o $@ -c $<

%.hex:
	@mkdir -p $(BUILD_DIR)
	$(OBJCOPY) $(OBJFLAGS) $(BUILD_DIR)/$(*).elf $(BUILD_DIR)/$@

clean:
	@rm -rf bin
