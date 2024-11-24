all:
	pio test --without-uploading --without-testing
clean:
	cd ../make && ./codegen.py -s
distclean: clean
	rm -rf `find .. -name .pio`
