# adapted from http://thedestitutedeveloper.blogspot.com/2014/07/msp430f5529-launchpad-project01.html

TARGET	= sensor_node
MCU	= msp430fr5739
SOURCES	= main.c spi.c delay.c uart.c

DEPEND = $(SOURCES:.c=.d)
OBJECTS = $(SOURCES:.c=.o)

SUPPORT_FILE_DIRECTORY = /opt/ti/mspgcc/include

INCLUDES = -Iinclude
CFLAGS   = -I $(SUPPORT_FILE_DIRECTORY) -mmcu=$(MCU) -g -Os -Wall -Wunused $(INCLUDES)
LDFLAGS  = -L $(SUPPORT_FILE_DIRECTORY) -mmcu=$(MCU) -T $(MCU).ld

CC       = msp430-elf-gcc
LD       = msp430-elf-ld
AR       = msp430-elf-ar
AS       = msp430-elf-gcc
NM       = msp430-elf-nm
OBJCOPY  = msp430-elf-objcopy
RANLIB   = msp430-elf-ranlib
STRIP    = msp430-elf-strip
SIZE     = msp430-elf-size
READELF  = msp430-elf-readelf
MAKETXT  = srec_cat
CP       = cp -p
RM       = rm -f
MV       = mv

all: $(TARGET).elf

$(TARGET).elf: $(OBJECTS)
	$(CC) $(CLFAGS) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@
	$(SIZE) $(TARGET).elf
	echo

%.o: %.c
	echo "Compiling $<"
	$(CC) -c $(CFLAGS) -o $@ $<

# include the dependencies unless we're going to clean, then forget about them.
ifneq ($(MAKECMDGOALS), clean)
-include $(DEPEND)
endif

%.d: %.c
	echo "Generating dependencies $@ from $<"
	$(CC) -M ${CFLAGS} $< >$@
.SILENT:
.PHONY: clean
clean:
	-$(RM) $(OBJECTS)
	-$(RM) $(TARGET).elf
	-$(RM) $(SOURCES:.c=.lst)
	-$(RM) $(DEPEND)

install: $(TARGET).elf
	mspdebug tilib "prog $(TARGET).elf"
