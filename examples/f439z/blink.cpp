#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "console.h"

constexpr Pin led (LED);  // defined in platformio.ini

int main () {
    hardFaulter = hardFaultHandler;
    fastClock();

    uint32_t stack [300];
    sys::init(stack); // enable multi-threading

    Uart uart ('U');
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
    consoleWriter = uartWriter<'U'>;

    printf("%s: blink @ %u MHz\n", SVDNAME, SystemCoreClock/1'000'000);

    led.mode("P");  // push-pull output
    while (true) {
        led.toggle();
        sys::wait(250);
    }
}
