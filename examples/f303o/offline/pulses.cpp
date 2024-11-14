// Process pulse runs reported via "R track" logs.

#include <cstdio>
#include <cstdlib>

// 10: bin count is template argument
template< uint16_t BINS >
struct Convolution {
    enum { HALF=BINS/2,NLOW=HALF,NHIGH=BINS/10,NBOTH=NLOW+NHIGH,PAD=BINS/100 };
    //enum { HALF=500,NLOW=500,NHIGH=100,NBOTH=600,PAD=5 };

    bool sig [BINS] {};
    int high =0, low =0, max =0, pos =0, offset =0, start =0, avg =0;
    bool inSync =false;

    int convolve (bool val, int num) {
        sig[num%BINS] = val;
        // track low count in (-600,-100] ms and high count in (-100,0] ms
        low += wrap(num-NHIGH) - wrap(num-NBOTH);
        high += wrap(num-0) - wrap(num-NHIGH);
        return (NLOW - low) + 5 * high;
    }

    bool feed (bool val, int num) {
        auto sum = convolve(val, num);
        if (sum > max) {
            max = sum;
            pos = num;
        }
        if (!inSync) {
            inSync = num-offset > 3*BINS && max > BINS*9/10 && sum < max*9/10;
            if (inSync) {
                offset = start = pos;
                avg = 0;
                fprintf(stderr, "sync %d\n", pos);
            }
        } else {
            auto rel = (num-offset+HALF)%BINS;
            if (rel == HALF-PAD)
                max = 0;
            else if (rel == HALF+PAD && max > BINS*8/10) {
                auto diff = (pos-start+HALF)%BINS-HALF;
                auto gap = num - offset;
                if (gap > 30*BINS) {
                    inSync = false;
                    max = 0;
                } else
                    printf("%d,%d,%d,%d,%d\n",
                            (num+HALF)%BINS, diff+HALF, max,
                            gap/100, avg/BINS+HALF);
                offset = pos;
                avg = (255*avg + diff*BINS) / 256;
                return true;
            }
        }
        return false;
    }

    int wrap (int n) const {
        return sig[(n+BINS)%BINS];
    }
};

// read stream from stdin, see stream.cpp for info about the RLE encoding
template< typename T >
void feeder () {
    T conv;
    int rept, signal, count = 0, peaks = 0;
    while (scanf("R track d%d m0 elapsed %d ", &signal, &rept) == 2) {
        for (auto r = 0; r < rept; ++r)
            peaks += conv.feed(signal, count++);
    }
    fprintf(stderr, "%d peaks in %.3f seconds\n", peaks, count/1000.0);
};

int main () {
    freopen("out.csv", "w", stdout);
    freopen("ticks.txt", "r", stdin);
    feeder<Convolution<250>>();
}
