// Decode DCF77 with the new worker design

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "decoder.h"

struct Event {
    uint32_t eDst :8;
    uint32_t eTag :24;

    Event (int dst, int tag =0) : eDst (dst), eTag (tag) {}
};

struct Worker {
    uint8_t wId =++wSeq;

    Worker () {
        assert(wId < sizeof workers / sizeof *workers);
        workers[wId] = this;
    }

    ~Worker () {
        workers[wId] = {};
    }

    virtual Event process (Event req, void* arg, Event reply) =0;

    inline static void dispatch (Event req, void* arg ={}, Event reply =0) {
        auto n = req.eDst;
        assert(n < sizeof workers / sizeof *workers && workers[n] != nullptr);
        workers[n]->process(req, arg, reply);
    }

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

struct Dcf77 : Worker {
    Decoder d;

    void init () {
        STK[0x4] = (SystemCoreClock/256) / 8 - 1; // reload value for ≈256 Hz
        STK[0x8] = 0;
        STK[0x0] = 0b011; // enable, clk/8 mode
    }

    Event process (Event, void*, Event reply) override {
        d.step(led);
        return reply;
    }
};

Dcf77 app;

// this needs "-DMYSYSTICK" to disable JeeH's default SysTick handler
extern "C" void SysTick_Handler () {
    led = dcfData;
    Worker::dispatch(app.wId);
}

int main() {
    initBoard();

    app.init();
    while (true)
        asm ("wfi");
}
