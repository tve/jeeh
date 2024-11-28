CXXFLAGS = -std=c++17 -Wall -Wextra -Wpedantic -Werror -Wno-format

all: conv capt.gz
	gzip -d <capt.gz | ./conv

deco: decode
	./decode

track: pulses
	./pulses

clean:
	rm -f conv decode pulses
