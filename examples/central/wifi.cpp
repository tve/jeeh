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

    // there are always two pending read requests
    while (true) {
        auto& m = sys::recv();
        assert(m.mTag == 'R');
        assert(m.mDst == uart.dId || m.mDst == uart_w.dId);

        uint8_t other = m.mDst ^ uart.dId ^ uart_w.dId;
        Message out { other, 'W', m.mLen, m.mPtr };
        sys::call(out);
 
        sys::send(m);
    }
}
