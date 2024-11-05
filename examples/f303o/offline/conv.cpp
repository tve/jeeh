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

// 4: show actual signal vs convolution result for a 10-sec range
struct Convolution_4 : Convolution_1 {
    int offset =0;

    bool feed (bool val, int num) {
        auto sum = convolve(val, num);
        if (sum > prev) {
            prev = sum;
            pos = num;
        }
        if (offset > 0 && num - offset < 10000)
            printf("%d,%d,%d\n", num-offset, 1000*val+600, sum);
        if (sum <= 1500)
            return false;
        // now at a fairly close match
        if (offset == 0 && num > 2500 && num > pos+50) {
            fprintf(stderr, "sync at %d ms\n", num);
            offset = pos; // found a good starting peak
        }
        return true;
    }
};

// 5: capture decoded DCF77 bit stream, relative to the convolution peaks
struct Convolution_5 : Convolution_1 {
    int offset =0, avg =-1, head =0, tail =0;
    uint64_t bits =0;

    bool feed (bool val, int num) {
        auto sum = convolve(val, num);
        if (sum > prev) {
            prev = sum;
            pos = num;
        }
        if (avg >= 0) {
            auto rel = (num - offset - avg/100 + 1500) % 1000;
            //auto rel = (num - avg/100 + 1500) % 1000;
            if (901 <= rel && rel <= 999) {
                head += val;
                if (rel == 999) {
                    if (head < 49)
                        printf("\n");
                    head = 0;
                }
            } else if (1 <= rel && rel <= 99) {
                tail += val;
                if (rel == 99) {
                    printf("%d", tail >= 49);
                    tail = 0;
                }
            }
        }
        if (sum <= 1500)
            return false;
        // now at a fairly close match
        if (avg < 0 && (num - offset) > 2500 && num > pos+50) {
            fprintf(stderr, "sync at %d ms\n", num);
            offset = pos; // found a good starting peak
            avg = 100 * 500;
        }
        if (avg >= 0 && (num - offset) % 1000 == 50) {
            auto rel = (pos - offset + 500) % 1000;
            if (450 < rel && rel < 550)
                avg = (99 * avg + 100 * rel) / 100;
            else if (num - offset > 10'000) {
                avg = -1;
                offset = num;
            }
            prev = 0;
        }
        return true;
    }
};

// 6: not sure this makes sense, trying to calculate the synchronised drift
struct Convolution_6 : Convolution_1 {
    int offset =0, avg =-1, head =0, tail =0;
    uint64_t bits =0;

    bool feed (bool val, int num) {
        auto sum = convolve(val, num);
        if (sum > prev) {
            prev = sum;
            pos = num;
        }
        if (sum <= 1500)
            return false;
        // now at a fairly close match
        if (avg < 0 && (num - offset) > 2500 && num > pos+50) {
            fprintf(stderr, "sync at %d ms\n", num);
            offset = pos; // found a good starting peak
            avg = 100 * 500;
        }
        if (avg >= 0 && (num - offset) % 1000 == 50) {
            auto rel = (pos - offset + 500) % 1000;
            auto diff = (prev + 1000) % 1000;
            printf("%d,%d,%d\n", (num-offset)/1000, diff, avg/100);
            if (450 < rel && rel < 550)
                avg = (99 * avg + 100 * rel) / 100;
            else if (num - offset > 10'000) {
                avg = -1;
                offset = num;
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
        case 4:  feeder<Convolution_4>(); break;
        case 5:  feeder<Convolution_5>(); break;
        case 6:  feeder<Convolution_6>(); break;
        default: fprintf(stderr, "oops, try: T=1 make\n");
    }
}
