template< uint16_t BINS, bool GAP >
struct Convolution {
    enum { BANDS = GAP ? 2 : 4 };

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
                //logf("sync %d", pos);
            }
        } else {
            auto rel = (num-off+BINS)%BINS;
            // look for an edge in the first 550 ms of each second
            if (sum > max && (BINS*24/50) < rel && rel < (BINS*26)/50) {
                max = sum;
                pos = num;
            }
            if (rel == BINS-1) {
                for (auto i = 0; i < BANDS; ++i) {
                    auto on = high[i] > BINS/20;
                    if (GAP) { // DCF sends bits in low-to-high order
                        bits[i] >>= 1;
                        bits[i] |= ((uint64_t) (on)) << 59;
                    } else { // MSF sends bits in high-to-low order
                        bits[i] <<= 1;
                        bits[i] &= (1ULL<<60) - 1; // keep bottom 60 bits
                        bits[i] |= on;
                    }
                }
                if (max > BINS*7/10) {
                    inSync = num-off < 10*BINS;
                    off = pos;
                }
                max = 0;
                // DCF has a missing pulse, MSF has a long pulse
                return GAP ? high[0] < BINS/20 && high[1] < BINS/20 :
                             high[0] > BINS/20 && high[1] > BINS/20 &&
                             high[2] > BINS/20 && high[3] > BINS/20;
            }
        }
        return false;
    }

    int check () const {
        auto v = bits[1];
        if constexpr (GAP)
            return ((int) v & 0x01) * 0x10 +
                   ((int) (~v>>20) & 0x01) * 0x08 +
                   (parity((v>>21) & 0xFF)) * 0x04 +
                   (parity((v>>29) & 0x7F)) * 0x02 +
                   (parity((v>>36) & 0x7FFFFF));
        else
            return 0; // TODO
    }

    // decode DCF or MSF bitstream, but set seconds to the weekday (1..7)
    DateTime decode () const {
        auto v = bits[1];
        if constexpr (GAP)
            return {
                bcd((v>>50) & 0xFF), bcd((v>>45) & 0x1F), bcd((v>>36) & 0x3F),
                bcd((v>>29) & 0x3F), bcd((v>>21) & 0x7F),
                (uint8_t) (v>>42) & 0x07
            };
        else
            return {
                bcd((v>>36) & 0xFF), bcd((v>>31) & 0x1F), bcd((v>>25) & 0x3F),
                bcd((v>>16) & 0x3F), bcd((v>>9) & 0x7F),
                (uint8_t) (v>>22) & 0x07
            };
    }

private:
    int wrap (int n, int s) const {
        return sig[(n+(BINS*s)/10)%BINS]; // careful with int truncation
    }

    int parity (uint32_t v) const {
        return __builtin_parity(v);
    }

    uint8_t bcd (int n) const {
        return n - 6 * (n>>4);
    }
};
