#include <jee.h>
#include <jee/cycles.h>
#include <jee/exti.h>
using namespace jeeh;
#include "defs.h"

ExtIrq exti;
EXTIRQ_INSTALL(exti)

struct RefClock : Worker {
    enum TAG { START, DONE };

    volatile uint32_t count =0;
    Pin ref {RTC_REFIN};

    Event process (Event in, Event out, void*) override {
        switch (in.eTag) {
            case START:
                ref.mode("D");
                exti.enable(ref, exti.RISE, DONE);
                break;
            case DONE:
                ++count;
                break;
            default:
                fail();
        }
        return out;
    }
};

int main () {
    initBoard();

    exti.init();

    RefClock worker;
    auto wkId = worker.init();

    Worker::send({ wkId, worker.START });

    while (true) {
        logf("%d", worker.count);
        cycles::msBusy(1000);
    }
}
