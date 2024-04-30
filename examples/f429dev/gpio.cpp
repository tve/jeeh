// Simple I2C sensor readout example.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

constexpr Pin ledL (LED1), ledR (LED2), ledC (LED3), ledB (LED4);

void jeeh::fail(void const* a, char const* f, int n) {
    printf("\nfailed at %s:%d\nfailed caller: %p\n", f, n, a);
    while (true) {}
}

void jeeh::hardFaultHandler (uint32_t* sp) {
    printf("\nhard fault, sp = %p", sp);
    fail();
}

Uart uart ('U');

extern "C" int _write (int, char* ptr, int len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

int main () {
    hardFaulter = hardFaultHandler;
    fastClock();
    cycles::init();

    ledL.mode("P");
    ledR.mode("P");
    ledC.mode("P");
    ledB.mode("P");

    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });

    printf("%s @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);

    { // I2C bus PH5+PH4
        I2cGpio i2c;
        printf("SCL=PH5, SDA=PH4\n");
        i2c.init("H5,H4");
        i2c.detect(); // look for audio codec
    }
    { // I2C bus PH8+PH7
        I2cGpio i2c;
        printf("SCL=PH8, SDA=PH7\n");
        i2c.init("H8,H7");
        i2c.detect(); // look for audio codec
    }
    { // SPI flash memory
        SpiGpio spi;
        SpiFlash spif (spi);

        spi.init("B15,B14,B13,B12");
        auto t = cycles::count();
        int id = spif.devId();
        t = cycles::count() - t;
        printf("spi: id %06x, %d kB, %d cycles\n", id, spif.size(), t);
        spi.deinit();
    }

    while (true) {
        ledL.toggle();  // red, left
        sys::wait(500);
        ledR.toggle();  // red, right
        ledC.toggle();  // red, on CPU board
        ledB.toggle();  // lcd backlight
    }
}
