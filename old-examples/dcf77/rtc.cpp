// Decode DCF77 using 256 Hz RTC steps, without adjusting for drift

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

uint32_t clickOffset;

uint32_t clicks () {
    return rtc::getDate().todTicks();
}

void stepSync () {
    clickOffset = clicks();
}

void stepWait (uint32_t n) {
    assert(n > 1);
    while ((clicks() - clickOffset) % n == 0) {}
    while ((clicks() - clickOffset) % n != 0) {}
}

int synchronise () {
    constexpr auto NUM = 64, STEP = 256 / NUM;

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

uint8_t fromBcd (uint8_t v) {
    return v - 6 * (v>>4);
}

DateTime decode () {
    uint64_t bits = 0;

    auto count = [](uint32_t ms) {
        constexpr auto MS = 2;
        int n = 0;
        for (auto j = 0U; j < ms; j += MS) {
            led = +dcfData;
            n += MS * led;
            stepWait(MS);
        }
        return n;
    };

    for (auto i = 0; ; ++i) {
        auto done = count(24) < 10;
        bits = (bits >> 1) | ((uint64_t) (count(26) > 14) << 59);
        count(206);

        if (done && i >= 59) {
            logf("%8d: %x%08x", i, (uint32_t) (bits>>32), (uint32_t) bits);
            break;
        }
    }

    // uint8_t yr, mo, dy, hh, mm, ss, ff;
    return { fromBcd((bits >> 50) & 0xFF),
             fromBcd((bits >> 45) & 0x1F),
             fromBcd((bits >> 36) & 0x3F),
             fromBcd((bits >> 29) & 0x3F),
             fromBcd((bits >> 21) & 0x7F) };
}

int main () {
    initBoard();

    auto n = synchronise();
    logf("wait %d clicks", n);
    stepSync();
    stepWait(n);
    stepSync();

    while (true) {
        auto dt = decode();
        logf("20%02d-%02d-%02d %02d:%02d", dt.yr, dt.mo, dt.dy, dt.hh, dt.mm);
    }
}
