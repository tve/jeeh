all:
	for i in */; do (echo $$i; cd $$i && pio run -s); done
clean:
	cd ../make && ./codegen.py -s
distclean: clean
	rm -rf `find .. -name .pio`
