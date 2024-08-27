// Decode DCF77 signal, without adjusting for any drift

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

uint32_t msOffset;

uint32_t myMillis () {
    //return rtc::getDate().todMillis();
    return cycles::millis();
}

void stepSync () {
    msOffset = myMillis();
}

void stepWait (uint32_t ms) {
    while ((myMillis() - msOffset) % ms == 0) {}
    while ((myMillis() - msOffset) % ms != 0) {}
}

int synchronise () {
    constexpr auto NUM = 50, STEP = 1000 / NUM;

    char tally [NUM+1];
    memset(tally, '0', NUM);
    tally[NUM] = 0;

    auto max = 0, pos = 0;

    stepSync();
    for (auto i = 0; i < 10; ++i) {
        for (auto j = 0; j < NUM; ++j) {
            led = +dcfData;
            tally[j] += led;
            if (tally[j] > max) {
                pos = j;
                max = tally[pos];
            }
            stepWait(STEP);
        }
        logf("%2d: %s", i+1, tally);
    }

    logf("%*c", 5 + pos, '^');
    return STEP * pos;
}

DateTime decode () {
    uint64_t bits = 0;

    auto count = [](uint32_t ms) {
        constexpr auto MS = 5;
        int n = 0;
        for (auto j = 0U; j < ms; j += MS) {
            led = +dcfData;
            n += MS * led;
            stepWait(MS);
        }
        return n;
    };

    for (auto i = 0; i < 200; ++i) {
        auto hi = count(100);
        if (hi < 50) {
            logf("%d #%d: %07x %08x",
                    i, hi, (uint32_t) (bits>>32), (uint32_t) bits);
            if (i >= 59)
                break;
        }
        hi += count(100);

        bits >>= 1;
        bits |= (uint64_t) (hi > 170) << 58;

        hi += count(800);
        //logf("high %d ms", hi);
    }

    auto fromBcd = [](uint8_t v) {
        return v - 6 * (v>>4);
    };

    DateTime dt { fromBcd((bits >> 50) & 0xFF),
                  fromBcd((bits >> 45) & 0x1F),
                  fromBcd((bits >> 36) & 0x3F),
                  fromBcd((bits >> 29) & 0x3F),
                  fromBcd((bits >> 21) & 0x7F) };
    logf("20%02d-%02d-%02d %02d:%02d", dt.yr, dt.mo, dt.dy, dt.hh, dt.mm);
    return dt; // uint8_t yr, mo, dy, hh, mm, ss, ff =0;
}

int main () {
    initBoard();

    stepSync();
    for (auto i = 0; i < 10; ++i) {
        stepWait(100);
        logf("t %04d ms", cycles::millis());
    }

    while (true) {
        auto ms = synchronise();
        logf("t %d ms, wait %d ms", myMillis(), ms);
        stepSync();
        stepWait(ms);

        cycles::clear();
        stepSync();

        auto dt = decode();
        logf("dt %u", +dt);
    }
}
