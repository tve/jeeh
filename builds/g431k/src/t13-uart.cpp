#include <jee.h>
#include <jee/uart.h>
using namespace jeeh;
#include "defs.h"
#include "test.h"

Uart uart;

int myThread (Message&) {
    logf("20");
    Message m { uart.dId, 'R' };
    sys::send(m);
    logf("21");
    while (true) {
        sys::recv();
        logf("22 %p #%d = '%.*s'", m.mPtr, m.mLen, m.mLen, m.mPtr);
    }
}

int main () {
    Tester t;

    uint32_t stack [300];
    sys::init(stack);

    logf("10");
    uart.init(Uart::Config{ UART_PINS, 115200, UART_NAME.ADDR,
                ena::UART_NAME, UART_FREQ, Irq::UART_NAME, UART_CONF });

    logf("30");
    Message m { uart.dId, 'W', 6, (uint8_t*) "Hello!" };
    sys::call(m);
    logf("31");

fail();
    uint32_t myStack [200];
    [[maybe_unused]] auto& my = sys::fork(myStack, myThread);

    logf("11");
    sys::wait(5);
    logf("12");
    sys::wait(10);
    logf("13");
    sys::wait(20);
    logf("14");

    auto& r = sys::recv();
    assert(&r == &my);
    logf("14 %d", r.mArg);
    assert(r.mArg == 12345);
}
