#include <jee.h>
using namespace jeeh;

int main () {
    fastClock();
    Pin led (LED);
    led.mode("P");

    while (true) {
        led.toggle();
        itmWrite("xy\n", 3);
        for (auto i = 0; i < 10'000'000; ++i)
            asm ("");
    }
}
