// Trivial LED blink demo.

#include <jee.h>

PinA<15> led;

int main () {
    led.mode(Pinmode::out);
    enableSysTick();

    while (true) {
        led = !led;
        wait_ms(500);

        //for (int i = 0; i < 1000000; ++i)
        //    __asm(""); // avoid getting optimised away
    }
}
