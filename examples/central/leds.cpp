// Switch each of the MikroBUS pins on and off for a quick LED-check.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

constexpr Pin dio0 ("A4");
constexpr Pin dio1 ("B0");
constexpr Pin dio2 ("B11");
constexpr Pin dio3 ("H4");
constexpr Pin dio5 ("H5");

int main () {
    initBoard("leds"); // in defs.h

    Pin pins [12];
    Pin::config("A4:P,F11,I0,I1,C2,C3,"
                "B0,B11,F6,F7,H4,H5", pins, sizeof pins);

    while (true)
        for (auto i = 0U; i < sizeof pins; i++) {
            pins[i] = 1;
            sys::wait(100);
            pins[i] = 0;
        }
}
