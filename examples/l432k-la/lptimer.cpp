// Explore an event-based low-power timer implementation.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

namespace jeeh::nvic {
void clearPend (uint8_t num) {
    NVIC[0x180 + 4*(num/32)] = 1 << num % 32; // clear pending
}
}

struct LpTimer {
    enum { ISR=0x00, ICR=0x04, IER=0x08, CFGR=0x0C, CR=0x10, ARR=0x18 };

    static constexpr auto TIM = LPTIM1;

    void init () const {
        RCC(ena::LPTIM1,1) = 1;
        RCC[0x88](18,2) = 3; // use LSE clock
        TIM[IER](1) = 1; // ARRMIE
        TIM[CR](0) = 1; // ENABLE
    }

    //void deinit () const { RCC(ena::LPTIM1,1) = 0; }

    void enable (uint16_t count) const {
        TIM[ARR] = count;
        TIM[CR](1) = 1; // SNGSTRT
    }

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

LpTimer timer;

int main () {
    initBoard();

    timer.init();

    auto i = 0;
    while (true) {
        led.toggle();

        logf("hi! %d", ++i); // uses blocking polled I/O

        timer.enable(1<<13); // 0.25 sec
        assert(!timer.done());
        asm ("wfe");
        assert(timer.done());

        timer.enable(1<<12); // 0.125 sec
        assert(!timer.done());
        cycles::msBusy(250);
        assert(timer.done());
        asm ("wfe"); // the event flag still needs to be cleared!
    }
}
