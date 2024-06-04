A = $(shell etc/attached.py)
G = grep --line-buffered -v -e 'Library Manager' \
			    -e ' SUCCESS   00:' \
			    -e '^pio-extra: GOT '

# run tests for all attached boards
all: first $A clean
	date >>LASTRUN.txt

$A:
	@ make -C $@ clean stub
	@ (pio run -d $@ -t check -s 2>&1 && echo PASS) | $G | tee $@.log
	@ grep -H -e ' FAILED ' -e '^PASS' $@.log >>LASTRUN.txt

examples:
	for i in ../examples/*/; do echo $$i:; pio run -d $$i -s || true; done

attached:
	etc/attached.py

first:
	date >LASTRUN.txt

clean:
	pio run -d g431k -e t00 -s
	cd ../make && ./codegen.py -s
	rm -rf `find .. -name .pio`

.PHONY: all examples attached first clean $A
