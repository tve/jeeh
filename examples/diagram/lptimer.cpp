// Explore an event-based low-power timer implementation.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

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
        while (!TIM[ISR](4)) {} // ARROK
        TIM[ICR] = 1<<4; // ARROKCF
        TIM[CR](1) = 1; // SNGSTRT
    }

    bool done () const {
        uint32_t isr = TIM[ISR];
        TIM[ICR] = isr; // clear all current interrupts
        if (isr & (1<<1)) { // ARRM
            // TODO create a utility function for this
            auto num = (uint8_t) Irq::LPTIM1;
            NVIC[0x180 + 4*(num/32)] = 1 << num % 32; // clear pending
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
#if 1
        logf("hi! %d", ++i); // this starts a DMA-based UART TX
        asm ("wfi"); // wait for uart dma completion so it won't interfere
        asm ("sev; wfe"); // clear event flag (is this caused by the DMA IRQ?)
#endif
        timer.enable(1<<14); // 0.5 sec
        assert(!timer.done());
        asm ("wfe");
        assert(timer.done());
    }
}
