// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "tick"

//CG1 board leds
#define LED  "A5"

const Pin led (LED,"P");

namespace serio {
    enum { ISR=0x00, TDR=0x04, BRR=0x08, CR1=0x0C };

    void init () {
        Pin::config("A2:U7");
        RCC(ena::USART2,1) = 1;
        USART2[BRR] = SystemCoreClock / 1'000'000;
        USART2[CR1] = (1<<13) | (1<<3); // UE TE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART2[ISR](7)) {} // TXE
            USART2[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!USART2[serio::ISR](6)) {} // TC
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
    //fastClock(); // 72 MHz
    rtc::init(false);

    serio::init();

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (dt.ff * 1000) / 256);
}
