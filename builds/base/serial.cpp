#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

static void delayLoop (uint16_t ms) {
    cycles::init();
    while (cycles::count() < ms * (SystemCoreClock/1000)) {}
}

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

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1)
        serio::write(buf, len);
    return len;
}

int main () {
    fastClock(); // 160 MHz
    serio::init();

    Pin led ("B8","P");

    uint32_t us = 0, bd = 0, seq = 0;

    while (true) {
        seq = seq % 64 + 1;
        cycles::init();
        printf("%2d us %5d kbd %*c\n", us, bd, seq, '#');
        us = cycles::micros();
        bd = (seq+17) * 10'000 / us;

        led = 1;
        delayLoop(50);
        led = 0;
        delayLoop(50);
    }
}
