// Decode DCF77 using a convolution kernel, called from the SysTick IRQ.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "decoder.h"

struct Event {
    uint8_t eDst =0;
    uint8_t eTag =0;
    uint16_t eLen =0;
};

struct Worker {
    uint8_t wId =++wSeq;

    Worker () {
        assert(wId < sizeof workers / sizeof *workers);
        workers[wId] = this;
    }

    virtual void process (Event to, Event from ={}, void* arg ={}) =0;

#if 0
    static Worker& worker (Event e) {
        auto id = e.eDst;
        assert(id < sizeof workers / sizeof *workers);
        auto p = workers[id];
        assert(p != nullptr);
        return *p;
    }
#endif

private:
    inline static uint8_t wSeq;
    inline static Worker* workers [20];
};

struct Trigger {
    uint8_t tId =++tSeq;

    Trigger () {
        assert(tId < sizeof triggers / sizeof *triggers);
        triggers[tId] = this;
    }

    virtual void interrupt () =0;

private:
    inline static uint8_t tSeq;
    inline static Trigger* triggers [20];
};

struct Dcf77 : Worker, Trigger {
    Decoder d;

    void process (Event, Event ={}, void* ={}) override {
        led = dcfData;
        d.step(led);
    }

    void interrupt () override {
        process({ wId });
    }
};

Dcf77 app;

// this needs "-DMYSYSTICK" to disable JeeH's default SysTick handler
extern "C" void SysTick_Handler () {
    led = dcfData;
    app.interrupt();
}

int main() {
    initBoard();

    STK[0x4] = (SystemCoreClock/256) / 8 - 1; // reload value for ≈256 Hz
    STK[0x8] = 0;
    STK[0x0] = 0b011; // enable, clk/8 mode

    while (true)
        asm ("wfi");
}
