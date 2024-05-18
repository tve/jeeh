A = f103r f302r f439z f723d g431k l433r l475d l496d wl55r
G = grep --line-buffered -v -e 'Library Manager' \
			    -e ' SUCCESS   00:' \
			    -e '^pio-extra: GOT '

all: first $A clean
	tail -n1 *.log | grep -v '^$$'
	date >>LASTRUN.txt

first:
	date >LASTRUN.txt

$A:
	make -C $@ clean stub
	(pio run -d $@ -t check -s 2>&1 && echo PASS: `date`) | $G | tee $@.log
	grep -H -e ' FAILED ' -e '^PASS' $@.log >>LASTRUN.txt

clean:
	pio run -d g431k -e t00 -s
	cd ../make && ./codegen.py -s
	rm -rf */.pio

.PHONY: all $A clean
