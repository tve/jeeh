// Explore convolution of DCF77 signal sampled at 1 kHz.

#include <cstdio>

const int16_t capture [] = {
#include "conv.h"
};

struct Convolution {
    bool sig [1000] {};
    int cnt =0, high =0, low =0, prev =0, pos =0;

    int wrap (int n) const {
        return sig[(n+1000) % 1000];
    }

    void feed (bool f) {
        auto idx = ++cnt % 1000;
        sig[idx] = f;
        // track low count in (-900,-100] and high count in (-100,0]
        low += wrap(idx-100) - wrap(idx-900);
        high += wrap(idx-0) - wrap(idx-100);
        auto sum = (800 - low) + 8 * high;
        if (sum > prev)
            pos = cnt;
        else if (pos == cnt-1 && prev > 1500)
            printf("%d,%d\n", pos%1000, prev);
        prev = sum;
    }
};

int main () {
    Convolution conv;
    for (auto e : capture)
        for (auto i = 0; i < 16; ++i)
            conv.feed((e >> i) & 1);
}
