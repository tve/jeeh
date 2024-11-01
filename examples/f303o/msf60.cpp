// Capture and decode MSF60 pulses.

#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/uart.h>
using namespace jeeh;
#include "defs.h"

template< uint16_t N >
struct Decoder {
    static_assert(N % 10 == 0);

    uint64_t rings [6];
    uint16_t next =0, ticks =0;
    uint8_t counts [N], hits;
    bool maxed;

    bool step (bool pin) {
        hits += pin; // count hits in current 100 ms period

        if (next % (N/10) == 0) { // on 100 ms boundary
            auto b = next / (N/10);
            if (b < 6) {
                auto& r = rings[b];
                r = (r<<1) | (2*hits >= N/10); // more than 50% on in 100 ms
                if (b == 4 && (r&1) != 0)
                    ticks = 0; // minute mark
            }
            hits = 0;
        }

        auto c = counts[next] += pin;
        maxed |= c >= 10;

        if (++next < N)
            return false; // not on seconds boundary

        next = 0;

        if (maxed) {
            // find start of peak, search from 200 ms backwards
            for (auto i = 0U; i < N; ++i) {
                auto t = ((12 * N / 10) - i) % N; // from 20% backwards
                if (counts[t] >= 8)
                    next = (N - t) % N;
            }

            memset(counts, 0, sizeof counts);
            maxed = false;
        }

        if (++ticks >= 60) {
            ticks = 0;
            return true;
        }
        return false;
    }

    // decode MSF bitstream, but return with seconds set to the weekday (1..7)
    DateTime msfDecode () const {
        auto bcd = [](int n) -> uint8_t { return n - 6*(n>>4); };
        auto r = rings[2];
        return {
            bcd((r>>35) & 0xFF), bcd((r>>30) & 0x1F), bcd((r>>24) & 0x3F),
            bcd((r>>15) & 0x3F), bcd((r>>8) & 0x7F), (uint8_t) (r>>21) & 0x07
        };
    }
};

int main () {
    initBoard();
    msfVcc = 1;

    Decoder<50> d;

    while (true) {
        cycles::msBusy(20);
        led = !msfDat;
        if (d.step(led)) {
            auto dt = d.msfDecode();
            logf("20%02d-%02d-%02d %02d:%02d %c",
                    dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, "?mtwhfsS"[dt.ss]);
            for (auto i = 0; i < 6; ++i) {
                char bits [61];
                for (auto j = 0; j < 60; ++j) {
                    auto b = (d.rings[i] >> j) & 1;
                    bits[j] = b ? '#' : j % 5 == 0 ? '+' : '-';
                }
                bits[60] = 0;
                logf("  %d: %s+", i, bits);
            }
        }
    }
}
