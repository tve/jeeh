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

extern void allTests ();

int main () {
    fastClock();
    cycles::init();

    UNITY_BEGIN();
    allTests();
    return UNITY_END();
}

// End of boilerplate, below is for temporary code, to be moved once ready.

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

    uint8_t init () {
        assert(wId == 0);
        wId =++wSeq;
        assert(wId < sizeof workers / sizeof *workers);
        workers[wId] = this;
        return wId;
    }

    static void send (Event evt, Event reply ={}, void* arg =nullptr) {
        assert(evt.eDst != 0);
        // TODO must postpone call when sending to a lower-priority worker!
        auto prev = current;
        current = &at(evt.eDst);
        reply = current->process(evt, reply, arg);
        current = prev;
        if (reply.eDst != 0)
            send(reply); // won't recurse again (i.e. can't reply to a reply)
    }

    static Worker& at (uint8_t id) {
        assert(0 < id && id < sizeof workers / sizeof *workers);
        assert(workers[id] != nullptr);
        return *workers[id];
    }

    static inline Worker* current;

protected:
    virtual Event process (Event in, Event out, void* arg) =0;
private:
    static inline uint8_t wSeq;
    static inline Worker* workers [20];

    static inline Event pool [100];
    static inline uint8_t free;
};
