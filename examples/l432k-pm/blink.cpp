// Blink the LED, using busy loops for delays.

#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

int main () {
    constexpr Pin led ("B3");
    led.mode("P");

    while (true) {
        led.toggle();
        cycles::msBusy(250);
    }
}
