#include <jee.h>
#include <jee/cycles.h>
#include <jee/ticker.h>
using namespace jeeh;
#include "defs.h"

Pin dcfDat ("A4","U");

Ticker ticker;
TICKER_INSTALL(ticker)

static void delayLoop (uint16_t ms) {
    cycles::init();
    while (cycles::count() < ms * (SystemCoreClock/1000)) {}
}

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
    ticker.init();

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);

    Pin vcc ("A6","P"), gnd ("A7","P"), pon ("B7","P");
    vcc = 1;
}

int main () {
    initBoard();

    led.toggle();
    delayLoop(100);

    while (true)
        led = dcfDat;
}
