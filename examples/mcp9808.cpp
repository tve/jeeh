// Periodically read out a MCP9808 temp sensor attached to i@c and blink LED.

#include <jee.h>
#include <jee/i2c-mcp9808.h>

UartDev< PinA<9>, PinA<10> > console;

int printf(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); veprintf(console.putc, fmt, ap); va_end(ap);
    return 0;
}

I2cBus< PinB<7>, PinB<6> > i2c;                   // standard I2C pins for SDA and SCL
MCP9808< decltype(i2c) > sensor;                  // std. chip on TvE's JZ4/JZ5

PinA<15> led4;          // LED on JZ4 pcb, active low
PinC<15> led5;          // LED on JZ5 pcb, active low

template< typename T >
void detectI2c (T bus) {
    for (int i = 0; i < 128; i += 16) {
        printf("%02x:", i);
        for (int j = 0; j < 16; ++j) {
            int addr = i + j;
            if (0x08 <= addr && addr <= 0x77) {
                bool ack = bus.start(addr<<1);
                bus.stop();
                printf(ack ? " %02x" : " --", addr);
            } else
                printf("   ");
        }
        printf("\r\n");
    }
}

int main() {
    led4.mode(Pinmode::out);
    led4 = 0;
    led5.mode(Pinmode::out);
    led5 = 0;

    uint32_t hz = fullSpeedClock();
    console.init();
    console.baud(115200, hz);
    wait_ms(10);
    printf("\r\n===== MCP9808 demo starting =====\r\n\n");

    i2c.init();
    detectI2c(i2c);

    if (!sensor.init()) {
        printf("\r\nOOPS, can't init MCP9808!\r\n");
        while(1) ;
    }
    wait_ms(sensor.convert());

    while (true) {
        int32_t t;
        if (t = sensor.read())
            printf("temp: %d.%02dC\r\n", t/100, t%100);

        led4 = 1-led4;
        led5 = 1-led5;

        wait_ms(1000);
    }
}

// sample output:
//
