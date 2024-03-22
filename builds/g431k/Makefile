E = t01-itm
X = -e /examples/
O = ~/.platformio/packages/tool-openocd
C = date; PLATFORMIO_DISABLE_COLOR=true time

all: clean
	git ls-files ../.. | grep  -v $X | entr -c make all-run
all-run:
	@ $C pio run -t check -s

check:
	git ls-files ../.. | grep  -v $X | entr -c make check-run E=$E
check-run:
	@ $C pio run -t clean -t check -s -e $E

openocd:
	$O/bin/openocd
clean:
	rm -rf .pio
strip:
	cd ../../make; ./codegen.py -s
