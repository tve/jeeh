// Decode DCF77 using a convolution kernel (ported from an older project)

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

struct Decoder {
    // convolution kernel: N1 x -1, N2 x 2, N3 x 0, N4 x -1 (NN is noise window)
    static constexpr auto N1 = 22, N2 = 22, N3 = 36, N4 = 22,
                          NK = N1+N2+N3+N4, NN = 256-N2-N3-2;

    uint32_t lastMin {};
    int16_t pulse {}, value {}, noise {}, noiseSum {};
    int16_t convMax {};
    uint8_t convOff {}, minPos {};
    int8_t minute [60] {};

    uint8_t match {}, good {}, flags [5] {
        0x55, 0x55, 0x55, 0x55, 0x55, // R A1 Z1 Z2 A2
    };

    template< typename T, unsigned N >
    struct Circular {
        static constexpr auto mask = N-1;
        static_assert((N&mask) == 0, "N must be a power of 2");

        // indexing support
        T operator[] (int n) const { return buf[(n+off)&mask]; }
        T& operator[] (int n) { return buf[(n+off)&mask]; }

        void next () { off = (off+1) & mask; }

        void rescale () { for (auto& e : buf) e /= 2; }

        int sum (int pos, int len) const {
            int t = 0;
            while (--len >= 0)
                t += operator[](pos++);
            return t;
        }

        int off {0};
        T buf [N] {};
    };

    Circular<uint8_t,256> counts;

    void step (int signal) {
        counts.next();
        if (signal) {
            if (++counts[0] == 255) {
                counts.rescale();
                convMax /= 2;
            }
        }

        auto v = convolution();
        if (v > convMax) {
            convMax = v;
            convOff = counts.off;
        }

        uint8_t n = counts.off - convOff;

        if (signal) {
            if (256-N4-N3-N2 <= n && n < 256-N4-N3)
                ++pulse;
            else if (256-N4-N3+7 <= n && n < 256-N4-7)
                ++value;
            if (n < 256-N4-N3-N2-1 || n >= 256-N4+1)
                ++noise;
        }

        if (n != N3 + N4)
            return; // not on a second boundary

        noiseSum += noise;

        auto mid = (noise*NN/256+2*22)/3; // 1's weighted twice as much as 0's
        if (minPos < sizeof minute)
            minute[minPos++] = value-mid;

        if (pulse < 5)
            completeMin();

        pulse = value = noise = 0;
    }

private:
    void completeMin () {
        uint64_t frame = 0;
        for (int i = 0; i < 60; ++i)
            frame = (frame << 1) | (minute[59-i] >= 0);
        minPos = 0;

        auto pty = dcfCheck(frame);
        auto dt = dcfDecode(frame);
        auto min = dt / 60;
        auto wday = dt.ss;

        match <<= 1;
        match |= min == ++lastMin;

        good <<= 1;
        good |= pty == 0;

        if (pty == 0) {
            lastMin = min;

            uint8_t events = frame >> 15; // bits 15..19
            for (auto& e : flags) {
                e <<= 1;
                e |= events & 1;
                events >>= 1;
            }
        }

        //setDate(dt, lastMin/(24*60));
        //setTime(dt, lastMin*60);

// 2021-09-12 14:20 S <2429E914A090C92> r00 a0000 zFF00 o45 n000 p00 mFF gFF
        logf("20%02d-%02d-%02d %02d:%02d %c <%07x%08x> "
                "r%02x a%02x%02x z%02x%02x o%02x n%03d p%02x m%02x g%02x\n",
                dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, "?mtwhfsS"[wday],
                (uint32_t) (frame>>33), (uint32_t) (frame>>1),
                flags[0], flags[1], flags[4], flags[2], flags[3],
                convOff, (999*noiseSum+60*NN-1)/(60*NN), pty, match, good);

        noiseSum = 0;
    }

    int convolution () {
        int t = 0;
        for (int i = 0; i < NK; ++i)
            t += pulseKernel(i) * counts[i-NK+1];
        return t;
    }

    static int pulseKernel (int i) {
        return i < 0 ? 0 :
               i < N1 ? -1 :
               i < N1+N2 ? 2 :
               i < N1+N2+N3 ? 0 :
               i < N1+N2+N3+N4 ? -1 : 0;
    }

    static int parity (uint32_t v) {
        return __builtin_parity(v);
    }

    static int dcfCheck (uint64_t b) {
        return ((int)  (b>> 0) & 0x01) * 0x10 +
               ((int) (~b>>20) & 0x01) * 0x08 +
               (parity((b>>21) & 0xFF)) * 0x04 +
               (parity((b>>29) & 0x7F)) * 0x02 +
               (parity((b>>36) & 0x7FFFFF));
    }

    // decode DCF bitstream, but return with seconds set to the weekday (1..7)
    static DateTime dcfDecode (uint64_t b) {
        auto bcd = [](int n) -> uint8_t { return n - 6*(n>>4); };
        return {
            bcd((b>>50) & 0xFF), bcd((b>>45) & 0x1F), bcd((b>>36) & 0x3F),
            bcd((b>>29) & 0x3F), bcd((b>>21) & 0x7F), (uint8_t) (b>>42) & 0x07
        };
    }
};

int main() {
    initBoard();

    Decoder d;

    int ssr = 0;
    while (true) {
        // wait until the next 256 Hz tick
        while (ssr == RTC[0x08]) {} // sub-second register
        ssr = RTC[0x08];

        led = dcfData;
        d.step(led);
    }
}
