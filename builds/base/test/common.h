// Common code, used in all tests.

#include <unity.h>
#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

// allow the use of printf and logf

extern "C" int _write (int, char* ptr, int len) {
    for (auto i = 0; i < len; ++i)
        putchar(ptr[i]);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

extern void allTests ();

int main () {
    fastClock();
    cycles::init();

    // adjust priorities before they might interfere with "real" IRQs
    //SCB.byte(0x1F) = 0xFF; // irq #11: SVC
    SCB.byte(0x22) = 0xFF; // irq #14: PendSV

    UNITY_BEGIN();
    allTests();
    return UNITY_END();
}

// End of boilerplate, below is for temporary code, to be moved once ready.

// this needs "-DMYSYSTICK" to disable JeeH's default handlers
#define IRQ_HANDLER(name, func) \
    extern "C" void name##_Handler () { func(); }

struct Event {
    uint32_t eDst :8;
    uint32_t eTag :8;
    uint32_t eVal :16;

    Event (uint8_t dst =0, uint8_t tag =0, uint16_t val =0)
        : eDst (dst), eTag (tag), eVal (val) {}
};

struct Worker {
    Worker () {}
    ~Worker () { workers[wId] = nullptr; }

    uint8_t init () {
        if (wId == 0) {
            wId =++wSeq;
            assert(wId < sizeof workers / sizeof *workers);
            workers[wId] = this;
        }
        return wId;
    }

    static void send (Event evt, Event done ={}, void* arg =nullptr) {
        assert(evt.eDst != 0);
        dispatch(evt, done, arg);
    }

    static void reply (Event evt) {
        if (evt.eDst != 0)
            dispatch(evt);
    }

    static void resetAll () { // to reset between test cases
        for (auto& e : workers)
            if (e != nullptr) {
                e->wId = e->wHead = 0;
                e = nullptr;
            }
        wSeq = 0;
    }

    static uint8_t level () {
        return current == nullptr ? 0 : current->wId;
    }

    static Worker& at (uint8_t id) {
        assert(0 < id && id < sizeof workers / sizeof *workers);
        assert(workers[id] != nullptr);
        return *workers[id];
    }

    static void irqPendSV () {
        assert(irqState() == 0); // PendSV magic ...

        for (auto e : workers)
            if (e != nullptr)
                while (e->wHead != 0)
                    dispatch(e->pull());
    }

protected:
    uint8_t wId =0;
    static inline Worker* current;

    virtual Event process (Event in, Event out, void* arg) =0;

    void trigger (uint8_t tag, uint16_t val =0) {
        assert(irqState() != 0); // can only be called from an IRQ handler
        pend({ wId, tag, val });
    }

private:
    uint8_t wHead =0;

    static inline uint8_t wSeq;
    static inline Worker* workers [20];
    static inline Event wPool [100];
    static inline uint8_t wFree;

    static uint32_t irqState () {
        uint32_t ipsr;
        asm ("mrs %0, ipsr" : "=r" (ipsr));
        return ipsr;
    }

    static void dispatch (Event evt, Event done ={}, void* arg =nullptr) {
        // TODO must postpone call when sending to a lower-priority worker!
        auto prev = current;
        current = &at(evt.eDst);
        reply(current->process(evt, done, arg));
        current = prev;
    }

    void pend (Event e) {
        assert(wFree < 100); // XXX
        auto next = ++wFree;
        // TODO pull from a free list
        wPool[next] = Event (wHead, e.eTag, e.eVal);
        wHead = next;
        SCB[0x04](28) = 1; // ICSR PENDSVSET
    }

    Event pull () {
        assert(wHead != 0);
        auto h = wPool[wHead];
        wHead = h.eDst;
        h.eDst = wId;
        // TODO return slot to the free list
        return h;
    }
};

extern "C" [[gnu::naked]]
void PendSV_Handler () {
    asm (
        " mrs r0,psr \n"
        " push {r0,lr} \n"
        " sub sp,#32 \n"
        " addw r0,pc,#16 \n"
        " str r0,[sp,#24] \n"
        " ldr r0,=0x01000000 \n"
        " str r0,[sp,#28] \n"
        " ldr r0,=0xFFFFFFF9 \n"
        " mov lr,r0 \n"
        " bx lr \n"
        " bl %0 \n" // target of addw above
        " svc 0 \n"
        " b . \n"   // never reached
    :: "i" (Worker::irqPendSV));
}

extern "C" [[gnu::naked]]
void SVC_Handler () {
    asm (
        " tst lr,#0x10 \n"
        " ite eq \n"
        " addeq sp,#104 \n"
        " addne sp,#32 \n"
        " pop {r0,r1} \n"
        " msr psr,r0 \n"
        " bx r1 \n"
    );
}
