#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

Pin pins [8];

int main () {
    Pin led ("B8","P");

    Pin::config("A15,B7,B5,B4,A11,B3,A1,A0", pins, sizeof pins);
    Pin::config(":P,,,,,,,", pins, sizeof pins);

    while (true) {
        led.toggle();

        for (auto i = 0U; i < sizeof pins; ++i) {
            pins[i] = 1;
            cycles::msBusy(10);
            pins[i] = 0;
        }
    }
}
