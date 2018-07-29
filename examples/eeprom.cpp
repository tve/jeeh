// Test built-in EEPROM

#include <jee.h>

UartDev< PinA<9>, PinA<10> > console;

int printf(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); veprintf(console.putc, fmt, ap); va_end(ap);
    return 0;
}

int main () {

    EEPROM eeprom;
    for (int i = 0; i < 10; ++i) {
        printf("Writing EEPROM iteration %d\r\n", i);
        for (int a = 0; a < 2048; ++a) {
            int b = a ^ 0xa; // don't write sequentially
            uint8_t v = (b+i+(i<<4))^0xa;
            eeprom.write8(b, v);
        }
        for (int a = 0; a < 2048; ++a) {
            uint8_t v = eeprom.read8(a);
            uint8_t exp = (a+i+(i<<4))^0xa;
            if (v != exp) {
                printf("@%x got %x expected %x\r\n", a, v, exp);
            }
        }
    }
}
