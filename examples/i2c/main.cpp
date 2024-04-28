// This code can be compiled with either "arduino" or "cmsis" as PIO framework.
// I.e. "pio run -e l432k-arduino" or "pio run -e l432-cmsis".

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

enum { HMC5883=0x1E, ADXL345=0x53, ITG3200=0x68 };

constexpr Pin led ("B3"); // Nucleo-L432KC

void jeeh::fail(void const* a, char const* f, int n) {
    printf("\nfailed at %s:%d\nfailed caller: %p\n", f, n, a);
    while (true) {}
}

void jeeh::hardFaultHandler (uint32_t* sp) {
    printf("\nhard fault, sp = %p", sp);
    fail();
}

Uart uart ('U'); // DMA/IRQ-based UART device driver, id = 'U'

extern "C" int _write (int, char* ptr, int len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

int main () {
    hardFaulter = hardFaultHandler;
    fastClock();

    // the UART config comes from platformio.ini and is defined in defs.h
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });

    printf("???\r%s @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);
    led.mode("P"); // push-pull output

    I2cGpio i2c;

    printf("SDA=PB5, SCL=PB4\n");
    i2c.init("B5,B4");
    i2c.detect(); // look for audio codec

    while (true) {
        sys::wait(250);
        led.toggle();
    }
}
