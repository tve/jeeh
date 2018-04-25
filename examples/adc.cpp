// Analog input demo.

#include <jee.h>

UartDev< PinA<9>, PinA<10> > console;

int printf(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); veprintf(console.putc, fmt, ap); va_end(ap);
    return 0;
}

ADC<1> adc;
PinA<0> ana1;
PinA<1> ana2;

int main () {
    fullSpeedClock();
    adc.init();

    while (true) {
        printf("1: %d, 2: %d, temp: %d, vref: %d\n",
                adc.read(ana1), adc.read(ana2), adc.read(16), adc.read(17));
        wait_ms(500);
    }
}

// sample output:
//  1: 104, 2: 1759, temp: 1690, vref: 1476
//  1: 104, 2: 1749, temp: 1690, vref: 1475
//  1: 104, 2: 1746, temp: 1690, vref: 1475
//  1: 104, 2: 1750, temp: 1690, vref: 1475
//  1: 105, 2: 1758, temp: 1690, vref: 1475
