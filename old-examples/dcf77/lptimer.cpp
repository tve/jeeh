// Decode DCF77 using a convolution kernel, triggered from LPTIM1.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "decoder.h"

namespace jeeh::nvic {
    void clearPend (uint8_t num) {
        NVIC[0x180 + 4*(num/32)] = 1 << num % 32; // clear pending
    }
}

struct LpTimer {
    enum { ISR=0x00, ICR=0x04, IER=0x08, CFGR=0x0C, CR=0x10, ARR=0x18 };

    static constexpr auto TIM = LPTIM1;

    void init (uint16_t count) const {
        RCC(ena::LPTIM1,1) = 1;
        RCC[0x88](18,2) = 3; // use LSE clock
        TIM[IER](1) = 1; // ARRMIE
        TIM[CR](0) = 1; // ENABLE
        TIM[ARR] = count-1;
        TIM[CR](2) = 1; // CNTSTRT
    }

    //void deinit () const { RCC(ena::LPTIM1,1) = 0; }

    bool done () const {
        uint32_t isr = TIM[ISR];
        TIM[ICR] = isr; // clear all current interrupts
        if (isr & (1<<1)) { // ARRM
            nvic::clearPend((uint8_t) Irq::LPTIM1);
            return true;
        }
        return false;
    }
};

int main() {
    initBoard();

    Decoder d;

    LpTimer timer;
    timer.init(128); // 32768 Hz / 128 = 256 Hz

    while (true) {
        // wait until the next 256 Hz tick
        while (!timer.done()) {}

        led = dcfData;
        d.step(led);
    }
}
