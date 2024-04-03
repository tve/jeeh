#include <jee.h>
using namespace jeeh;
#include "defs.h"
#include "test.h"

int main () {
    Tester t;

    Pin pins [2];
    Pin::config(UART_PINS, pins, sizeof pins);
    pins[0].mode("P");
    pins[1].mode("F");

    assert(pins[1] == 0);
    pins[0] = 1;
    assert(pins[1] == 1);
}
