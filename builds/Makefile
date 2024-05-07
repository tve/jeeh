A = f103r f439z f723d g431k l433r l475d
G = grep --line-buffered -v 'Library Manager'

all: $A
	wc *.log
	tail -n1 *.log
	cd ../make && ./codegen.py -s

$A:
	make -C $@ clean stub
	pio run -d $@ -t check -s 2>&1 | $G | tee $@.log

.PHONY: $A
