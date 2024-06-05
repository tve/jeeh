#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Uart uart ('U');

extern "C" int _write (int, char* ptr, int len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

int main () {
    fastClock();
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });

    Pin led (LED);
    led.mode("P");
    while (true) {
printf("\n%s: hello @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);
        led.toggle();
        sys::wait(500);
    }
}
