// Process pulse runs reported via "R track" logs.

#include <cstdio>
#include <cstdlib>

template< uint16_t BINS, uint8_t BANDS >
struct Convolution {
    static_assert(1 <= BANDS && BANDS <= 5);

    bool sig [BINS] {}, inSync =false;
    int high[BANDS] ={}, low =0, max =0, pos =0, off =0, num =0;
    uint64_t bits [BANDS] {};

    bool feed (bool val) {
        ++num;
        // track low count in first 500 ms and high counts in next 5x 100 ms
        low += wrap(num, 5) - wrap(num, 0);
        sig[num%BINS] = val;
        for (auto i = 0; i < BANDS; ++i)
            high[i] += wrap(num, 6+i) - wrap(num, 5+i);
        auto sum = (BINS/2 - low) + 5 * high[0];

        if (!inSync) {
            if (sum > max) {
                max = sum;
                pos = num;
            }
            // synchronise once 3s have nee seen, a good edge was detected,
            // and the match has dropped a bit, i.e. we're past that edge
            inSync = num-off > 3*BINS && max > (BINS*9)/10 && sum < (max*9)/10;
            if (inSync) {
                off = pos;
                fprintf(stderr, "sync %d\n", pos);
            }
        } else {
            auto rel = (num-off+BINS)%BINS;
            // look for an edge in the first 550 ms of each second
            if (sum > max && rel < (BINS*11)/20) {
                max = sum;
                pos = num;
            }
            if (rel == BINS-1) {
                for (auto i = 0; i < BANDS; ++i) {
                    bits[i] >>= 1;
                    bits[i] |= ((uint64_t) (high[i] > BINS/20)) << 59;
                }
                if (max > BINS*7/10) {
                    inSync = num-off < 30*BINS;
                    off = pos;
                }
                max = 0;
                return high[0] < BINS/20;
            }
        }
        return false;
    }

    int wrap (int n, int s) const {
        return sig[(n+(BINS*s)/10)%BINS]; // careful with int truncation
    }
};

int main () {
    freopen("out.csv", "w", stdout);
    freopen("ticks.txt", "r", stdin);

    // read pulse runs from input file
    Convolution<250,5> conv;
    int signal, rept, match = 0;
    while (scanf("R track d%d m0 elapsed %d ", &signal, &rept) == 2)
        for (auto r = 0; r < rept; ++r)
            if (conv.feed(signal)) {
                for (auto i = 0U; i < 5; ++i)
                    printf(" %07x%08x", (uint32_t) (conv.bits[i]>>32),
                                        (uint32_t) conv.bits[i]);
                printf("\n");
                ++match;
            }
    fprintf(stderr, "%d matches in %.3f seconds\n", match, conv.num/1000.0);
}
