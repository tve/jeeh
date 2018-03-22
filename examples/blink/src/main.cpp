#include "jee.h"

JDigitalOut<PC_13> led;

void setup() {
    // put your setup code here, to run once:
}

void loop() {
    led = 0;
    wait_ms(100);
    led = !led;
    wait_ms(900);
}
