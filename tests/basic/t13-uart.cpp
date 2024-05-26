#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "test.h"

Uart uart ('U');

void uartWrite (void const* ptr, size_t len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
}

int myThread (Message&) {
    logf("20");
    auto n = 0;
    Message m { uart.dId, 'R' };
    do {
        sys::call(m);
        n += m.mLen;
    } while (m.mPtr[m.mLen-1] != '!');
    logf("21");
    return n;
}

int main () {
    Tester t;

    uint32_t stack [300];
    sys::init(stack);

#if STM32L0
    auto baud = 115'200;
#else
    auto baud = SystemCoreClock / 32; // i.e. 4'687'500 baud @ 150 MHz
#endif
    logf("10 %d", baud); itmFlush();
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
#if STM32L0
    assert(r.mArg == 71*71+4); // messed up by log output on same uart
#else
    assert(r.mArg == 71*71+1);
#endif
}
