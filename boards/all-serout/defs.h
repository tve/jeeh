// Lines with "CG" control the code-generated parts of this file.

//CG1 board leds
#define LED  "B3"

//CG2 board serout
#define SEROUT_DEV USART2
#define SEROUT_PIN "A2:7"

Pin led (LED, "P");

namespace serout {
#if SEROUT_VER
    enum { ISR=0x00, TDR=0x04, BRR=0x08, CR1=0x0C, UE=13 };
#else    
    enum { CR1=0x00, BRR=0x0C, ISR=0x1C, TDR=0x28, UE=0 };
#endif

    void init () {
        Pin::config(SEROUT_PIN);
        RCC(ena::SEROUT_DEV,1) = 1;
        SEROUT_DEV[BRR] = SystemCoreClock / 115'200;
        SEROUT_DEV[CR1] = (1<<3) | (1<<UE); // TE UE
    }

    void write (char const* ptr, int len) {
        while (--len >= 0) {
            while (!SEROUT_DEV[ISR](7)) {} // TXE
            SEROUT_DEV[TDR] = *ptr++;
        }
    }
}

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        serout::write(buf, len);
    return len;
}
