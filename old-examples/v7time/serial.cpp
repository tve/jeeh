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

int main () {
    fastClock(); // 72 MHz
    serio::init();

    Pin led ("A5","P");

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
