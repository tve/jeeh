#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

constexpr Pin pin0 ("A15"), pin1 ("B7"), pin2 ("B5"), pin3 ("B4"),
              pin4 ("A11"), pin5 ("B3"), pin6 ("A1"), pin7 ("A0");

Pin pins [] = { pin0, pin1, pin2, pin3, pin4, pin5, pin6, pin7 };

struct Replier : Worker {
    Event process (Event, Event out) override {
        pin1 = 1; // 37 µs
        pin1 = 0; // 39 µs
        return out;
    }
};

struct Sender : Worker {
    enum TAG { START, REPLY };

    Event process (Event in, Event out) override {
        pin5 = 1; // 30 & 77 µs
        switch (in.eTag) {
            case START:
                pin6 = 1; // 31 µs
                send({ (uint8_t) in.eVal }, { wId, REPLY });
                pin6 = 0; // 56 µs
                break;
            case REPLY:
                pin7 = 1; // 78 µs
                pin7 = 0; // 80 µs
                break;
            default:
                fail();
        }
        pin5 = 0; // 57 & 81 µs
        return out;
    }
};

int main () {
    Pin::config(":P,,,,,,,", pins, sizeof pins);
    Pin led ("B8","P");

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

    pin0 = 1; // SCL
    pin4 = 1; // NSEL

    pin2 = 1; // 0 µs
    Replier replier;
    Sender sender;
    auto rId = replier.init();
    auto sId = sender.init();
    pin3 = 1; // 23 µs
    Worker::send({ sId, sender.START, rId });
    pin3 = 0; // 85 µs
    pin2 = 0; // 86 µs
        
    while (true) {}
}
