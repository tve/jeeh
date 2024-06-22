// Test external PSRAM memory.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#include <jee/util/memtest.h>

int main () {
    initBoard("wifi"); // in defs.h
    espPower(true); // this generates a greeting after a few seconds

    logf("Connected ...");

    Message uMsg { uart.dId, 'R' };
    sys::send(uMsg); // start read from console

    Message wMsg { uart_w.dId, 'R' };
    sys::send(wMsg); // start read from uart_w

    while (true) {
        // there are always two outstanding uart requests at this point:
        //  - either a pending read request for each uart
        //  - or a write of the received data to the other uart

        auto& m = sys::recv();
        assert(m.mTag == 'R' || m.mTag == 'W');
        assert(m.mDst == uart.dId || m.mDst == uart_w.dId);

        m.mTag ^= 'R' ^ 'W'; // switch from reading to writing and vice versa
        m.mDst ^= uart.dId ^ uart_w.dId; // set destination to the other uart

        sys::send(m);
    }
}
