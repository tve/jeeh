E = t01-itm
X = -e /examples/ -e /log/
O = ~/.platformio/packages/tool-openocd

all: clean
	git ls-files ../.. | grep  -v $X | entr -c time make all-run
all-run:
	@ date; pio run -t check -s

check:
	git ls-files ../.. | grep  -v $X | entr -c time make check-run E=$E
check-run:
	@ date; pio run -t clean -t check -s -e $E

openocd:
	$O/bin/openocd
clean:
	rm -rf .pio
strip:
	cd ../../make; ./codegen.py -s
