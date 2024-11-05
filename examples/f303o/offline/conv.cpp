// Explore convolution of DCF77 signal sampled at 1 kHz.

#include <cstdio>
#include <cstdlib>

// 1: find all peaks > 1500 and mark the ms offset in the second
struct Convolution_1 {
    bool sig [1000] {};
    int high =0, low =0, prev =0, pos =0;

    int convolve (bool val, int num) {
        sig[num % 1000] = val;
        // track low count in (-900,-100] and high count in (-100,0]
        low += wrap(num-100) - wrap(num-900);
        high += wrap(num-0) - wrap(num-100);
        return (800 - low) + 8 * high;
    }

    bool feed (bool val, int num) {
        auto sum = convolve(val, num);
        if (sum > prev)
            pos = num;
        else if (pos == num-1 && prev > 1500) {
            printf("%d,%d\n", pos%1000, prev);
            return true;
        }
        prev = sum;
        return false;
    }

    int wrap (int n) const {
        return sig[(n+1000) % 1000];
    }
};

// 2: find peak, but stick to fixed 1000 ms intervals to find each max
struct Convolution_2 : Convolution_1 {
    bool feed (bool val, int num) {
        auto sum = convolve(val, num);
        if (sum > prev) {
            prev = sum;
            pos = num;
        }
        bool step = num % 1000 == 999;
        if (step) {
            printf("%d,%d\n", pos%1000, prev);
            prev = 0;
        }
        return step;
    }
};

// 3: find peak within (950,1050), ignore all others (incl the 1-min mark)
struct Convolution_3 : Convolution_1 {
    int offset =0, avg =0;

    bool feed (bool val, int num) {
        auto sum = convolve(val, num);
        if (sum > prev) {
            prev = sum;
            pos = num;
        }
        if (sum <= 1500)
            return false;
        // now at a fairly close match
        if (offset == 0 && num > 2500 && num > pos+50) {
            fprintf(stderr, "sync at %d ms\n", num);
            offset = pos; // found a good starting peak
            avg = 100 * 500;
        }
        if (offset > 0 && (num - offset) % 1000 == 50) {
            auto rel = (pos - offset + 500) % 1000;
            if (450 < rel && rel < 550) {
                avg = (99 * avg + 100 * rel) / 100;
                printf("%d,%d,%d\n", pos, rel, avg/100);
            }
            prev = 0;
        }
        return true;
    }
};

// read stream from stdin, see stream.cpp for info about the RLE encoding
template< typename T >
void feeder () {
    T conv;
    int rept, bits, count = 0, peaks = 0;
    while (scanf("%d %d", &rept, &bits) == 2) {
        for (auto r = 0; r < rept; ++r)
            for (auto i = 0; i < 16; ++i)
                peaks += conv.feed((bits >> i) & 1, count++);
    }
    fprintf(stderr, "%d peaks in %.3f seconds\n", peaks, count/1000.0);
};

int main () {
    auto e = getenv("T");
    auto t = e != nullptr ? atoi(e) : 0;
    if (t > 0) {
        char buf [20];
        snprintf(buf, sizeof buf, "out-%d.csv", t);
        freopen(buf, "w", stdout);
    }

    switch (t) {
        case 1:  feeder<Convolution_1>(); break;
        case 2:  feeder<Convolution_2>(); break;
        case 3:  feeder<Convolution_3>(); break;
        default: fprintf(stderr, "oops, try: T=1 make\n");
    }
}
