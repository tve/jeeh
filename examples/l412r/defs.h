// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "dcf77"

//CG1 board leds
#define LED  "B13"

Pin led (LED,"P");

Pin dcfData {"C0","U"}, dcfPon {"C1","P"};

namespace serio {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, RDR=0x24, TDR=0x28 };

    void init () {
        Pin::config("A2:8");
        RCC(ena::LPUART1,1) = 1;
        // 1 Mbaud with extra ÷256 divider, but avoid 32-bit overflow
        LPUART1[BRR] = 4 * SystemCoreClock / (1'000'000 / 64);
        LPUART1[CR1] = (1<<3) | (1<<2) | (1<<0); // TE RE UE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!LPUART1[ISR](7)) {} // TXE
            LPUART1[TDR] = ((uint8_t const*) ptr)[i];
        }
        while (!LPUART1[serio::ISR](6)) {} // TC
    }
}

void initBoard () {
    cycles::init();
    rtc::init();
#if 1
    RCC[0x00](2) = 1; // MSIPLLEN
    fastClock(false); // 48 MHz MSI
    cycles::msBusy(3); // let MSI PLL stabilise
#else
    fastClock(); // 80 MHz
#endif

    serio::init();

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        serio::write(buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    serio::write(ptr, len);
}
