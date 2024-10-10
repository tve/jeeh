// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "lcd"

//CG1 board leds
#define LED  "A5"

Pin led (LED,"P");

namespace serio {
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, TDR=0x28 };

    void init () {
        Pin::config("A2:7");
        RCC(ena::USART2,1) = 1;
        USART2[BRR] = SystemCoreClock / 10'000'000; // 160 MHz CPU clock
        USART2[CR1] = (1<<29) | (1<<3) | (1<<0); // FIFOEN TE UE
    }

    void write (void const* ptr, int len) {
        for (auto i = 0; i < len; ++i) {
            while (!USART2[ISR](7)) {} // TXE
            USART2[TDR] = ((uint8_t const*) ptr)[i];
        }
        //while (!USART2[serio::ISR](6)) {} // TC
    }
}

void initBoard () {
    fastClock(); // 160 MHz
    serio::init();
    cycles::init();

    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        serio::write(buf, len);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}
