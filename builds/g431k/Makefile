E = t00
X = -e /examples/ -e /log/
O = ~/.platformio/packages/tool-openocd/bin
G = ~/.platformio/packages/toolchain-gccarmnoneeabi/bin

ifeq ($(shell uname -s),Linux)
T = time -f '   %e real   %U user   %S sys'
M = make --no-print-directory
else
T = time
M = make
endif

all: clean
	git ls-files ../.. | grep  -v $X | entr -c $T $M all-run
all-run:
	@ date; pio run -t check -s

check:
	git ls-files ../.. | grep  -v $X | entr -c $T $M check-run E=$E
check-run:
	@ date; pio run -t clean -t check -s -e $E

assert:
	pio run -e $E -s
	@ read -p "enter address: " addr; \
	  $G/arm-none-eabi-addr2line  -Cfe .pio/build/$E/firmware.elf $$addr
pull: strip
	git pull
sizes:
	pio run -s && $G/arm-none-eabi-size .pio/build/*/firmware.elf
openocd:
	$O/openocd
stub:
	cd stub && pio run -t upload
clean:
	rm -rf .pio
strip:
	cd ../../make && ./codegen.py -s

.PHONY: stub
