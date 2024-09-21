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
    uint8_t wId =0;
    uint8_t head =0;

    Worker () {}
    ~Worker () { workers[wId] = nullptr; }

    uint8_t init () {
        if (workers[wId] == nullptr) {
            wId =++wSeq;
            assert(wId < sizeof workers / sizeof *workers);
            workers[wId] = this;
        }
        return wId;
    }

    static void send (Event evt, Event reply ={}, void* arg =nullptr) {
        assert(evt.eDst != 0);

        if (irqState() != 0) { // postpone when called from an IRQ
            assert(reply.eDst == 0 && arg == nullptr); // only replies allowed
            at(evt.eDst).pend(evt);
        } else
            dispatch(evt, reply, arg);
    }

    static void dispatch (Event evt, Event reply ={}, void* arg =nullptr) {
        // TODO must postpone call when sending to a lower-priority worker!
        auto prev = current;
        current = &at(evt.eDst);
        reply = current->process(evt, reply, arg);
        current = prev;
        if (reply.eDst != 0)
            dispatch(reply); // won't recurse again (can't reply to a reply)
    }

    static void clearAll () {
        memset(workers, 0, sizeof workers);
        wSeq = 0;
    }

    static Worker& at (uint8_t id) {
        assert(0 < id && id < sizeof workers / sizeof *workers);
        assert(workers[id] != nullptr);
        return *workers[id];
    }

    static void irqPendSV () {
        for (auto e : workers)
            if (e != nullptr && e->head != 0)
                dispatch(e->pull());
    }

    static inline Worker* current;

protected:
    virtual Event process (Event in, Event out, void* arg) =0;

    void pend (Event e) {
        assert(free < 100); // XXX
        auto next = ++free;
        // TODO pull from a free list
        pool[next] = Event (head, e.eTag, e.eVal);
        head = next;
        SCB[0x04](28) = 1; // ICSR PENDSVSET
    }

private:
    static uint32_t irqState () {
        uint32_t ipsr;
        asm ("mrs %0, ipsr" : "=r" (ipsr));
        return ipsr;
    }

    Event pull () {
        assert(head != 0);
        auto h = pool[head];
        head = h.eDst;
        h.eDst = wId;
        // TODO return slot to the free list
        return h;
    }

    static inline uint8_t wSeq;
    static inline Worker* workers [20];

    static inline Event pool [100];
    static inline uint8_t free;
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
        // target of addw above:
        " bl %0 \n"
        " svc 0 \n"
        " b . \n"
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
        " mov lr,r1 \n"
        " bx lr \n"
    );
}
