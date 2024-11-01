#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
#include <jee/uart.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    while (true) {
        logf("%d ms", rtc::getDate().todMillis());

        led = 1;
        cycles::msBusy(100);
        led = 0;
        cycles::msBusy(900);
    }
}
