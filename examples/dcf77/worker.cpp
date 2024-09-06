// Decode DCF77 with the new worker design

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "decoder.h"

struct Event {
    uint32_t eDst :8;
    uint32_t eTag :8;
    uint32_t eVal :16;

    Event (uint8_t dst =0, uint8_t tag =0, uint16_t val =0)
        : eDst (dst), eTag (tag), eVal (val) {}
};

struct Worker {
    uint8_t wId =++wSeq;
    uint8_t head =0;

    Worker () {
        assert(wId < sizeof workers / sizeof *workers);
        workers[wId] = this;
    }

    ~Worker () {
        workers[wId] = nullptr;
    }

    Event toSelf (uint8_t tag, uint16_t val =0) {
        return { wId, tag, val };
    }

    virtual Event process (Event req, Event reply, void* arg) =0;

    static Worker& at (uint8_t id) {
        assert(0 < id && id < sizeof workers / sizeof *workers);
        assert(workers[id] != nullptr);
        return *workers[id];
    }

    void pend (Event e) {
        auto next = ++free;
        // TODO pull(free);
        pool[next] = Event (head, e.eTag, e.eVal);
        head = next;
    }

    Event pull () {
        assert(head != 0);
        auto& curr = pool[head];
        head = curr.eDst;
        return Event (~0, curr.eTag, curr.eVal);
    }

    static inline Worker* current;
private:
    static inline uint8_t wSeq;
    static inline Worker* workers [20];

    static constexpr auto NPOOL = 100;
    static inline Event pool [NPOOL];
    static inline uint8_t free;
};

namespace jeeh::sys {
    void Xsend (Event req, Event reply ={}, void* arg =nullptr) {
        // FIXME this needs to POSTPONE if the priority is lower!
        //  and pick up all pending events before resuming
        Worker::at(req.eDst).process(req, reply, arg);
    }
}

struct Dcf77 : Worker {
    enum TAG { INIT, STEP };

    Decoder d; // see decoder.h, needs to be called 256x per second

    Event process (Event request, Event reply, void*) override {
        switch (request.eTag) {
            case INIT:
                // set up SysTick interrupts at ≈256 Hz
                STK[0x4] = (SystemCoreClock/256) / 8 - 1;
                STK[0x8] = 0;
                STK[0x0] = 0b011; // enable, clk/8 mode
                break;
            case STEP:
                led = request.eVal;
                d.step(request.eVal);
                break;
            default:
                fail(); // unknown request
        }
        return reply;
    }

    void interrupt () {
        sys::Xsend(toSelf(STEP, dcfData));
    }

};

Dcf77 app;

// this needs "-DMYSYSTICK" to disable JeeH's default SysTick handler
extern "C" void SysTick_Handler () {
    app.interrupt(); // TODO tie this into Ticker iso of this demo app
}

int main() {
    initBoard();

    sys::Xsend(app.toSelf(app.INIT));
    while (true)
        asm ("wfi");
}
