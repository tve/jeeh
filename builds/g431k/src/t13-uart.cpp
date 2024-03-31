#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "test.h"

Uart uart;

int myThread (Message&) {
    logf("20");
    Message m { uart.dId, 'R' };
    do {
        sys::send(m);
        logf("21");
        sys::recv();
        logf("22 %p #%d = '%.*s'", m.mPtr, m.mLen, m.mLen, m.mPtr);
    } while (m.mPtr[m.mLen-1] != '!');
    logf("23");
    return 0;
}

void uartWrite (void const* ptr, size_t len) {
    Message m { uart.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
}

int main () {
    Tester t;

    uint32_t stack [300];
    sys::init(stack);

    logf("10");
    uart.init(UART_PINS, 115200, { UART_NAME.ADDR, ena::UART_NAME,
                                   UART_FREQ, Irq::UART_NAME, UART_CONF });

#if 1
    logf("30");
    uartWrite("Hello\n", 6);
    logf("31");
    uartWrite(" Hiya\n", 6);
    logf("32");
    fail();
#endif

    uint32_t myStack [200];
    [[maybe_unused]] auto& my = sys::fork(myStack, myThread);

#define W(s) uartWrite(s "\n", sizeof s)
#include "pattern.h"
    uartWrite("!", 1); // final text quits myThread

    auto& r = sys::recv();
    assert(&r == &my);
    logf("14 %d", r.mArg);
    assert(r.mArg == 12345);
}
