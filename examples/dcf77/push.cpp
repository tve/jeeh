// Decode DCF77 by pushing the signal inta a decoder object

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

template< uint16_t NSLOTS >
struct Decoder {
    bool inSync;
    uint16_t slot;
    uint8_t count, errors;
    uint8_t bins [NSLOTS];
    uint64_t shift;

    Decoder () { resync(); }

    void resync () {
        logf("out of sync");
        inSync = false;
        slot = errors = 0;
        memset(bins, 0, sizeof bins);
    }

    bool step (bool on) {
        bins[slot] += on;
        slot = (slot+1) % NSLOTS;

        if (!inSync) {
            if (bins[slot] > 9) {
                logf("sync at %d of %d", slot, NSLOTS);
                slot = 0;
                inSync = true;
            }
            return false;
        }

        count += on;
        switch (slot) {
            case 0: // 0.0 s
                count = 0;
                break;
            case NSLOTS/10: // 0.1 s
                if (count < NSLOTS/20) { // missing pulse?
                    if (bits(0, 1) == 0 && bits(20, 1) == 1 &&
                        __builtin_popcount(bits(17, 2)) == 1 &&
                        __builtin_parity(bits(21, 8)) == 0 &&
                        __builtin_parity(bits(29, 7)) == 0 &&
                        __builtin_parity(bits(36, 23)) == 0) {
                        errors = 0;
                        return true;
                    }
                    logf("error: %08x %08x", bits(32, 28), (uint32_t) shift);
                    if (++errors > 10)
                        resync();
                }
                count = 0;
                break;
            case NSLOTS/5: // 0.2 s
                shift = (shift >> 1) | ((uint64_t) (count > NSLOTS/20) << 58);
                break;
        }

        return false;
    }

    uint32_t bits (uint8_t pos, uint8_t num) const {
        return (shift >> pos) & ((1 << num) - 1);
    }

    uint8_t bcd (uint8_t v) const {
        return v - 6 * (v>>4);
    }

    DateTime now () const {
        // uint8_t yr, mo, dy, hh, mm, ss, ff;
        return { bcd(bits(50, 8)), bcd(bits(45, 5)), bcd(bits(36, 6)),
                 bcd(bits(29, 6)), bcd(bits(21, 7)) };
    }
};

int main () {
    initBoard();

    Decoder<256> decoder;

    int ssr = 0;
    while (true) {
        // wait until the next 256 Hz tick
        while (ssr == RTC[0x28]) {} // sub-second register
        ssr = RTC[0x28];

        led = dcfData;
        ledN = !(decoder.inSync && decoder.slot == 0);

        if (decoder.step(led)) {
            auto dt = decoder.now();
            logf("20%02d-%02d-%02d %02d:%02d",
                    dt.yr, dt.mo, dt.dy, dt.hh, dt.mm);
        }
    }
}
