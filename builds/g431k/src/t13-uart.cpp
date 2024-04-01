#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "test.h"

Uart uart;

int myThread (Message&) {
    logf("20");
    auto n = 0;
    Message m { uart.dId, 'R' };
    do {
        sys::call(m);
        n += m.mLen;
    } while (m.mPtr[m.mLen-1] != '!');
    logf("22");
    return n;
}

void uartWrite (void const* ptr, size_t len) {
    Message m { uart.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
}

int main () {
    Tester t;

    uint32_t stack [300];
    sys::init(stack);

    auto baud = SystemCoreClock / 16; // i.e. 9.375 Mbaud @ 150 MHz
    logf("10 %d", baud);
    uart.init(UART_PINS, baud, { UART_NAME.ADDR, ena::UART_NAME,
                                 UART_FREQ, Irq::UART_NAME, UART_CONF });

    uint32_t myStack [200];
    [[maybe_unused]] auto& my = sys::fork(myStack, myThread);

#define W(s) uartWrite(s "\n", sizeof s)
#include "pattern.h"
    uartWrite("!", 1); // final text quits myThread

    auto& r = sys::recv();
    assert(&r == &my);
    logf("14 %d", r.mArg);
    assert(r.mArg == 71*71+1);
}
