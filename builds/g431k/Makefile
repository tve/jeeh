#X = -e /builds/ -e /tests/
X = -e /examples/
O = ~/.platformio/packages/tool-openocd

all:
	git ls-files ../.. | grep  -v $X | entr -cr pio run -t upload
openocd:
	$O/bin/openocd
