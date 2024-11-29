// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "tick"

//CG[ board leds
#define LED  "B0"
#define LED1 "B0"
#define LED2 "B7"
#define LED3 "B14"
//CG]

const Pin led (LED,"P");

namespace serio {
    enum { CR1=0x0C, BRR=0x08, ISR=0x00, TDR=0x04 };

    void init () {
        Pin::config("D8:U7");
        RCC(ena::USART3,1) = 1;
        USART3[BRR] = (SystemCoreClock/2) / 115'200;
        USART3[CR1] = (1<<13) | (1<<3); // UE TE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART3[ISR](7)) {} // TXE
            USART3[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!USART3[serio::ISR](6)) {} // TC
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
    fastClock(); // 64 MHz
    rtc::init();

    serio::init();

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, dt.ms);
}
