// Explore convolution of DCF77 signal sampled at 1 kHz.

#include <cstdint>
#include <cstdio>

const int16_t signal [] = {
#include "conv.h"
};

int main () {
    printf("%d %d\n", signal[0], sizeof signal * 16);
}
