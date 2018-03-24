#include <jee.h>

UartDev<Pin<'A',9>,Pin<'A',10>> uart;

void printf(const char* fmt, ...) {
    va_list ap;
    va_start (ap, fmt);
    veprintf(uart.putc, fmt, ap);
    va_end (ap);
}

// detect devices on an I2C bus

template< typename T >
void detectI2c (T bus) {
    for (int i = 0; i < 128; i += 16) {
        printf("%02x:", i);
        for (int j = 0; j < 16; ++j) {
            int addr = i + j;
            if (0x08 <= addr && addr <= 0x77) {
                bool ack = bus.start(addr);
                bus.stop();
                printf(ack ? " %02x" : " --", addr);
            } else
                printf("   ");
        }
        printf("\n");
    }
}

I2cDev<Pin<'B',7>,Pin<'B',6>> i2cbus;

int main () {
    detectI2c(i2cbus);
}

// sample output:
//
//  00:                         -- -- -- -- -- -- -- --
//  10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
//  20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
//  30: -- -- -- -- -- -- -- -- -- -- 3A 3B -- -- -- --
//  40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
//  50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
//  60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
//  70: -- -- -- -- -- -- -- --
