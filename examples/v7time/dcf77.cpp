#include <jee.h>
#include <jee/cycles.h>
#include <jee/ticker.h>
using namespace jeeh;
#include "defs.h"

Pin dcfData ("A4","U");

//Ticker ticker;
//TICKER_INSTALL(ticker)

namespace serio {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, TDR=0x28 };

    void init () {
        Pin::config("A9:7");
        RCC(ena::USART1,1) = 1;
        USART1[BRR] = SystemCoreClock / 2'000'000; // 72 MHz CPU clock
        USART1[CR1] = (1<<3) | (1<<0); // TE UE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART1[ISR](7)) {} // TXE
            USART1[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!USART1[serio::ISR](6)) {} // TC
    }
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        serio::write(buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    serio::write(ptr, len);
}

void initBoard () {
    fastClock(); // 72 MHz
    serio::init();
    cycles::init();
    rtc::init();
    //ticker.init();

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);

    Pin vcc ("A6","P"), gnd ("A7","P"), pon ("B7","P");
    vcc = 1;
}

uint32_t clickOffset;

uint32_t clicks () {
    return cycles::millis();
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

uint8_t fromBcd (uint8_t v) {
    return v - 6 * (v>>4);
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

    for (auto i = 0; ; ++i) {
        auto done = count(100) < 40;
        bits = (bits >> 1) | ((uint64_t) (count(100) > 60) << 59);
        count(800);

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
