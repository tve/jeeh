E = t01-itm
#X = -e /builds/ -e /tests/
X = -e /examples/
O = ~/.platformio/packages/tool-openocd

all:
	git ls-files ../.. | grep  -v $X | \
		entr -c time pio run -t upload -s
upload:
	git ls-files ../.. | grep  -v $X | \
		entr -c time pio run -t upload -s -e $E
openocd:
	$O/bin/openocd
