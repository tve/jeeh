#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

Pin pins [8];

struct Replier : Worker {
    Event process (Event, Event out, void*) {
        pins[1] = 1; // 37 µs
        pins[1] = 0; // 39 µs
        return out;
    }
};

struct Sender : Worker {
    enum TAG { START, REPLY };

    Event process (Event in, Event out, void*) {
        pins[5] = 1; // 30 & 77 µs
        switch (in.eTag) {
            case START:
                pins[6] = 1; // 31 µs
                send({ (uint8_t) in.eVal }, { wId, REPLY });
                pins[6] = 0; // 56 µs
                break;
            case REPLY:
                pins[7] = 1; // 78 µs
                pins[7] = 0; // 80 µs
                break;
            default:
                fail();
        }
        pins[5] = 0; // 57 & 81 µs
        return out;
    }
};

int main () {
    Pin led ("B8","P");

    Pin::config("A15,B7,B5,B4,A11,B3,A1,A0", pins, sizeof pins);
    Pin::config(":P,,,,,,,", pins, sizeof pins);

    for (auto i = 0U; i < 2; ++i) {
        led.toggle();

        for (auto j = 0U; j < sizeof pins; ++j) {
            pins[j] = 1;
            cycles::msBusy(3);
            pins[j] = 0;
        }
    }
    cycles::msBusy(10);

    for (auto e : pins)
        e = 1;
    for (auto e : pins)
        e = 0;
    cycles::msBusy(10);

    pins[0] = 1; // SCL
    pins[4] = 1; // NSEL

    pins[2] = 1; // 0 µs
    Replier replier;
    Sender sender;
    auto rId = replier.init();
    auto sId = sender.init();
    pins[3] = 1; // 23 µs
    Worker::send({ sId, sender.START, rId });
    pins[3] = 0; // 85 µs
    pins[2] = 0; // 86 µs
        
    while (true) {}
}
