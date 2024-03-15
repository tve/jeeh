E = g431k
X = -e /examples/ -e /tests/

compile:
	git ls-files ../.. | grep  -v $X | entr -c pio run -e $E
upload:
	git ls-files ../.. | grep  -v $X | entr -c pio run -e $E -t upload
debug:
	git ls-files ../.. | grep  -v $X | entr -c pio debug -e $E

gdb:
	pio debug -e $E --interface=gdb -- -q -x .pioinit
console:
	pio device monitor
logs:
	pio device monitor -p socket://127.0.0.1:6464 -f swo -f time

.PHONY: compile upload debug console gdb logs
