// Common code, used in all tests.

#include <unity.h>
#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

// avoid pulling in logf, etc
void jeeh::fail (void const*, char const*, int line) {
    TEST_ASSERT_EQUAL(0, line);
    __builtin_unreachable();
}

void jeeh::hardFaultHandler (uint32_t*) { fail(); }

// allow the use of printf
extern "C" int putchar (int ch);

extern "C" int _write (int, char* ptr, int len) {
    for (auto i = 0; i < len; ++i)
        putchar(ptr[i]);
    return len;
}

extern void allTests ();

int main () {
    fastClock();
    cycles::init();

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

        auto irq = inIrq();
        if (irq != 0 && irq != 14) { // postpone when called from an IRQ
            assert(reply.eDst == 0 && arg == nullptr); // only replies allowed
            at(evt.eDst).pend(evt);
            return;
        }

        accept(evt, reply, arg);
    }

    static void accept (Event evt, Event reply ={}, void* arg =nullptr) {
        // TODO must postpone call when sending to a lower-priority worker!
        auto prev = current;
        current = &at(evt.eDst);
        reply = current->process(evt, reply, arg);
        current = prev;
        if (reply.eDst != 0)
            send(reply); // won't recurse again (i.e. can't reply to a reply)
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

    static void dispatch () {
        for (auto e : workers)
            if (e != nullptr && e->head != 0)
                accept(e->pull());
    }

    static inline Worker* current;

protected:
    virtual Event process (Event in, Event out, void* arg) =0;

private:
    static uint32_t inIrq () {
        uint32_t ipsr;
        asm ("mrs %0, ipsr" : "=r" (ipsr));
        return ipsr;
    }

    void pend (Event e) {
        auto next = ++free;
        // TODO pull(free);
        pool[next] = Event (head, e.eTag, e.eVal);
        head = next;
        SCB[0x04](28) = 1; // ICSR PENDSVSET
    }

    Event pull () {
        assert(head != 0);
        auto h = pool[head];
        head = h.eDst;
        h.eDst = wId;
        return h;
    }

    static inline uint8_t wSeq;
    static inline Worker* workers [20];

    static inline Event pool [100];
    static inline uint8_t free;
};

IRQ_HANDLER(PendSV, Worker::dispatch)
