SDCC=sdcc
SDLD=sdcc
OBJECTS=sensors.ihx

.PHONY: all clean flash

all: $(OBJECTS)

clean:
	rm -f $(OBJECTS)

flash: sensors.ihx
	stm8flash -c stlink-v2 -pstm8s103?3 -w sensors.ihx

%.ihx: %.c stm8.h
	$(SDCC) -lstm8 -mstm8 --out-fmt-ihx $(CFLAGS) $(LDFLAGS) $<
	stm8flash -c stlinkv2 -p stm8s103?3 -w sensors.ihx
	# minicom -b 115200 -D /dev/ttyUSB4
