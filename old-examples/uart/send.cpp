// Simple message send over UART.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    int seq = 0;
    while (true) {
        for (auto i = 0; i < 7; ++i)
            printf("123456789.");
        logf(" %d", ++seq);

        sys::wait(250);
        led.toggle();
    }
}
