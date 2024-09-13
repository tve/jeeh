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

template< typename T, int N >
class RingBuffer {
    uint16_t in =0, out =0;
    T buf [N];
    static_assert((N & (N-1)) == 0, "must be a power of 2");

public:
    bool full () const {
        return (((in+1) ^ out) % N) == 0;
    }

    bool empty () const {
        return ((in ^ out) % N) == 0;
    }

    void put (T v) {
        assert(!full());
        buf[in++ % N] = v;
    }

    void putAtomic (T v) {
        assert(!full());
        auto i = __atomic_fetch_add(&in, 1, __ATOMIC_RELAXED);
        buf[i % N] = v;
    }

    T get () {
        assert(!empty());
        return buf[out++ % N];
    }

    T getAtomic () {
        assert(!empty());
        auto i = __atomic_fetch_add(&out, 1, __ATOMIC_RELAXED);
        return buf[i % N];
    }
};

uint32_t currIrq () {
    uint32_t ipsr;
    asm ("mrs %0, ipsr" : "=r" (ipsr));
    return ipsr;
}

void setPendSV () {
    SCB[0x04](28) = 1; // ICSR PENDSVSET
}

namespace jeeh::sys {
    void Xsend (Event req, Event reply ={}, void* arg =nullptr);
}

struct Worker {
    uint8_t wId =0;
    uint8_t head =0;

    Worker () {}

    void init () {
        assert(wId == 0);
        wId =++wSeq;
        assert(wId < sizeof workers / sizeof *workers);
        workers[wId] = this;
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

    void request (uint8_t tag, uint16_t val =0) const {
        assert(wId > 0);
        triggers.putAtomic({ wId, tag, val });
        setPendSV();
        asm ("isb"); // make sure PendSV runs now
    }

    static void dispatch () {
        while (!triggers.empty()) {
            auto req = triggers.get();
            // FIXME this needs to POSTPONE if the priority is lower!
            sys::Xsend(req);
        }
    }

    uint8_t level () const {
        return current == nullptr ? 0 : current->wId;
    }

    static inline Worker* current;
private:
    static inline uint8_t wSeq;
    static inline Worker* workers [20];

    static inline RingBuffer<Event,8> triggers;

    static inline Event pool [100];
    static inline uint8_t free;
};

namespace jeeh::sys {
    void Xsend (Event req, Event reply, void* arg) {
        // FIXME this needs to POSTPONE if the priority is lower!
        //  and pick up all pending events before resuming
        Worker::at(req.eDst).process(req, reply, arg);
    }
}

struct DCF77 : Worker {
    enum TAG { INIT, STEP };

    Decoder d; // see decoder.h, needs to be called 256x per second

    Event process (Event req, Event reply, void*) override {
        switch (req.eTag) {
            case INIT:
                // set up SysTick interrupts at ≈256 Hz
                STK[0x4] = (SystemCoreClock/256) / 8 - 1;
                STK[0x8] = 0;
                STK[0x0] = 0b011; // enable, clk/8 mode
                break;
            case STEP:
                led = req.eVal;
                d.step(req.eVal);
                break;
            default:
                fail(); // unknown request
        }
        return reply;
    }

    void interrupt () {
        request(STEP, dcfData);
    }

};

DCF77 app;

#define IRQ_HANDLER(name, func) \
    extern "C" void name##_Handler () { func(); }

// this needs "-DMYSYSTICK" to disable JeeH's default handlers
IRQ_HANDLER(SysTick, app.interrupt)
IRQ_HANDLER(PendSV, Worker::dispatch)

int main() {
    initBoard();

    app.init();
    app.request(app.INIT);

    while (true)
        asm ("wfi");
}
