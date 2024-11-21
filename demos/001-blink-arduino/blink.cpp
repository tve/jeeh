#include <Arduino.h>
const int LED = PC13;

void setup () {
    pinMode(LED, OUTPUT);
}

void loop () {
    digitalWrite(LED, LOW);
    delay(100);
    digitalWrite(LED, HIGH);
    delay(400);
}
