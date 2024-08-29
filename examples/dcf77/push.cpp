// Decode DCF77 by pushing the signal inta a decoder object

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

template< uint16_t NSLOTS >
struct Decoder {
    bool inSync;
    uint8_t count, pass, fail;
    uint8_t bins [NSLOTS];
    uint16_t slot;
    uint32_t history;
    uint64_t shifter;

    Decoder () { resync(); }

    void resync () {
        logf("out of sync");
        inSync = false;
        history = slot = pass = fail = 0;
        memset(bins, 0, sizeof bins);
    }

    bool step (bool on) {
        auto valid = false; // nothing to report, keep going

        bins[slot] += on;
        slot = (slot+1) % NSLOTS;

        if (inSync) {
            count += on;
            switch (slot) {
                case 0: // 0.0 s
                    count = 0;
                    break;
                case NSLOTS/10: // 0.1 s
                    if (count < NSLOTS/20) // missing pulse
                        valid = verify();
                    count = 0;
                    break;
                case 2*NSLOTS/10: // 0.2 s
                    shifter >>= 1;
                    shifter |= ((uint64_t) (count > NSLOTS/20) << 58);
                    break;
            }
        } else if (bins[slot] > 9) {
            logf("sync at %d of %d", slot, NSLOTS);
            slot = 0;
            inSync = true;
        }

        return valid;
    }

    bool verify () {
        if (bits(0, 1) == 0 && bits(20, 1) == 1 &&      // fixed
                __builtin_popcount(bits(17, 2)) == 1 && // dst
                __builtin_parity(bits(21, 8)) == 0 &&   // minute
                __builtin_parity(bits(29, 7)) == 0 &&   // hour
                __builtin_parity(bits(36, 23)) == 0) {  // date
            fail = 0;

            // only compare to previous if not start of the hour
            auto check = bits(29, 30);
            if (bits(21, 8) != 0) { // minute + parity
                // copy time to RTC after 5x same hour & date
                if (check == history) {
                    if (++pass == 5) {
                        logf("set RTC");
                        rtc::set(now());
                    }
                    return true; // shifter and now() are valid
                }
                pass = 0;
                auto dt = now();
                logf("rejecting: 20%02d-%02d-%02d %02d:%02d",
                        dt.yr, dt.mo, dt.dy, dt.hh, dt.mm);
            }
            history = check;
        } else if (++fail >= 10)
            resync();
        else
            logf("fail %d: %08x %08x",
                    fail, bits(32, 28), (uint32_t) shifter);
        return false;
    }

    uint32_t bits (uint8_t pos, uint8_t num) const {
        return (shifter >> pos) & ((1 << num) - 1);
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
