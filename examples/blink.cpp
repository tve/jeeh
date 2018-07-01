// Trivial LED blink demo.

#include <jee.h>

//PinA<15> led; // jnz1???
PinA<8> led; // jnz4

int main () {
    led.mode(Pinmode::out);
    led = 1;
    enableSysTick(2100000/1000);
    //fullSpeedClock();

    while (true) {
        led = !led;
        wait_ms(500);

        //for (int i = 0; i < 1000000; ++i)
        //    __asm(""); // avoid getting optimised away
    }
}
