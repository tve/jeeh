E = t01-itm
#X = -e /builds/ -e /tests/
X = -e /examples/
O = ~/.platformio/packages/tool-openocd

all: clean
	git ls-files ../.. | grep  -v $X | \
		entr -c time pio run -t check -s
check:
	git ls-files ../.. | grep  -v $X | \
		entr -c time pio run -t clean -t check -s -e $E
openocd:
	$O/bin/openocd
clean:
	rm -rf .pio
strip:
	cd ../../make; ./codegen.py -s
