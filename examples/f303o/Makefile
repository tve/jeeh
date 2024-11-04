CXXFLAGS = -std=c++17 -Wall -Wextra -Wpedantic -Werror -Wno-format

all: conv capt.gz
	gzip -d <capt.gz | ./conv
