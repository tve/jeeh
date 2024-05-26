A = f103r f439z f723d g431k l433r wl55r
B = f302r h743z l073r l475d l496d
G = grep --line-buffered -v -e 'Library Manager' \
			    -e ' SUCCESS   00:' \
			    -e '^pio-extra: GOT '

# allways run these tests by default
all-1: first $A clean
	tail -n1 *.log | grep -v '^$$'
	date >>LASTRUN.txt

# these boards are not always connected
all-2: $B clean
	tail -n1 *.log | grep -v '^$$'
	date >>LASTRUN.txt

# force a full test, assuming all boards are present
all: all-1 all-2

first:
	date >LASTRUN.txt

$A $B:
	@ make -C $@ clean stub
	@ (pio run -d $@ -t check -s 2>&1 && echo PASS) | $G | tee $@.log
	@ grep -H -e ' FAILED ' -e '^PASS' $@.log >>LASTRUN.txt

examples:
	for i in ../examples/*/; do echo $$i:; pio run -d $$i -s || true; done

# ignore the output files, re-fetch what's in git
drop-out:
	git checkout -- *-out/

clean:
	pio run -d g431k -e t00 -s
	cd ../make && ./codegen.py -s
	rm -rf */.pio ../examples/*/.pio

.PHONY: all all-1 all-2 examples drop-out clean $A $B
