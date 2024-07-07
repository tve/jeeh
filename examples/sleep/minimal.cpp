// Try out the automatic sleep modes.

#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

uint8_t jeeh::lowestPower (uint8_t, uint16_t) {
    return sys::STOP2;
}

void jeeh::resumePower () {
    //slowClock(false); // not needed, still running slow
}

int main () {
    slowClock(false);
    rtc::init(true);
    cycles::init();

    Pin led ("B3");
    led.mode("P");

    while (true) {
        led = 1;
        cycles::msBusy(1);
        led = 0;
        cycles::msBusy(1);
        sys::wait(500);
    }
}
