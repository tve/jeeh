// Minimal LED blinker, using a busy loop as delay.

#include <jee.h>
using namespace jeeh;

int main () {
    Pin led (LED);  // defined in platformio.ini
    led.mode("P");  // push-pull output

    while (true) {
        led.toggle();

        for (auto i = 0; i < 1'000'000; ++i)
            asm ("");  // prevents getting optimised away
    }
}
