// Decode DCF77 by pushing the signal inta a decoder object

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

template< uint16_t NSLOTS >
struct Decoder {
    bool inSync;
    uint16_t slot;
    uint8_t count;
    uint8_t bins [NSLOTS];
    uint64_t shifter;

    Decoder () { init(); }

    void init () {
        inSync = false;
        slot = 0;
        memset(bins, '0', sizeof bins);
    }

    bool step (bool on) {
        bins[slot] += on;
        slot = (slot+1) % NSLOTS;

        if (!inSync) {
            if (bins[slot] > '9') {
                // can't use logf since it truncates past 80 chars
                serio::write(bins, sizeof bins);
                logf(" = max %d @ %d", bins[slot]-'0', slot);

                inSync = true;
                slot = 0;
            }
            return false;
        }

        if (slot == 0 || slot == NSLOTS/10)
            count = 0;
        if (slot < NSLOTS/5)
            count += on;
        if (slot == NSLOTS/5)
            shifter = (shifter >> 1) | ((uint64_t) (count > NSLOTS/20) << 58);

        if (slot == NSLOTS/10-1 && count < NSLOTS/20)
            return true; // missing pulse
        return false;
    }

    uint8_t fromBcd (uint8_t pos, uint8_t mask) const {
        auto v = (shifter >> pos) & mask;
        return v - 6 * (v>>4);
    }

    DateTime now () const {
        // uint8_t yr, mo, dy, hh, mm, ss, ff;
        return { fromBcd(50, 0xFF),
                 fromBcd(45, 0x1F),
                 fromBcd(36, 0x3F),
                 fromBcd(29, 0x3F),
                 fromBcd(21, 0x7F) };
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
