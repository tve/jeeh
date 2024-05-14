A = f103r f302r f439z f723d g431k l433r l475d
G = grep --line-buffered -v -e 'Library Manager' \
			    -e ' SUCCESS   00:' \
			    -e '^pio-extra: GOT '

all: $A clean
	wc *.log
	tail -n1 *.log | grep -v '^$$'

$A:
	make -C $@ clean stub
	(pio run -d $@ -t check -s 2>&1 && echo PASS: $@) | $G | tee $@.log

clean:
	pio run -d g431k -e t00 -s
	cd ../make && ./codegen.py -s
	rm -rf */.pio

.PHONY: all $A clean
