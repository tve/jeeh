// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "tick"

//CG1 board leds
#define LED  "B0"

namespace serio {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, TDR=0x28 };

    void init () {
        Pin::config("D8:U7");
        RCC(ena::USART3,1) = 1;
        USART3[BRR] = SystemCoreClock / 1'000'000;
        USART3[CR1] = (1<<3) | (1<<0); // TE UE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART3[ISR](7)) {} // TXE
            USART3[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!USART3[serio::ISR](6)) {} // TC
    }
}

Pin led (LED,"P");

void initBoard () {
    //fastClock(); // 216 MHz
    cycles::init();
    rtc::init();

    serio::init();

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, dt.ms);
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        serio::write(buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    serio::write(ptr, len);
}
