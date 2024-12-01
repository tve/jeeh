// Minimal serial output example, blinking the LED as it loops.
// All board details are in "defs.h", using settings from "platformio.ini".

#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;
#include "defs.h"

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

void msDelay (int ms) {
#if STM32G0 | STM32L0
    // Cortex M0+ has no cycle counter and cycles::millis won't advance
    // ... but let's still implement a bit of a delay for the LED blinks
    ms *= SystemCoreClock / 5000;
    for (int i = 0; i < ms; ++i)
        asm ("");
#else
    cycles::msBusy(ms);
#endif
}

int main () {
    cycles::init();
    serout::init();

    while (true) {
        led = 1;
        msDelay(100);
        led = 0;
        msDelay(400);

        printf("%u ms\n", (unsigned) cycles::millis());
    }
}
