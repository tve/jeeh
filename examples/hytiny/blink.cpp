// Minimal LED blinker, using a busy loop as delay.

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

int main () {
    fastClock();
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
    printf("\n%s: blink @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);

    rtc::init(false);

    Pin led (LED);  // defined in platformio.ini
    led.mode("P");  // push-pull output

    while (true) {
        printf("%d\n", rtc::getSecs());
        led.toggle();
        sys::wait(500);
    }
}

void jeeh::fail (void const* a, char const* f, int n) {
    printf("%s:%d - called at %p\n", f, n, a);
    while (true) {}
}

void jeeh::LowPower::start (Message&) {}
void jeeh::LowPower::finish () {}
