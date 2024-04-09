A = f103r f439z f723d g431k l433r
G = grep --line-buffered -v 'Library Manager'

all: $A
	wc *.log

$A:
	make -C $@ clean stub
	make -C $@ all-run 2>&1 | $G | tee $@.log

.PHONY: $A
