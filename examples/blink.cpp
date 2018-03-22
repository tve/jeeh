#include <jee.h>

Pin<'C',13> led;

int main () {
    led.mode(Pinmode::out);

    while (true) {
        led = !led;

        for (int i = 0; i < 1000000; ++i)
            __asm("");
    }
}
