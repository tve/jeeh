// Create a UART <=> WiFi bridge (with the ESP8266 running Zimodem).

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#include <jee/util/memtest.h>

int main () {
    initBoard(); // in defs.h
    espPower(true); // this generates a greeting after a few seconds

    logf("Connected ...");

    Message uMsg { console.dId, 'R' };
    sys::send(uMsg); // start read from console

    Message wMsg { uart_w.dId, 'R' };
    sys::send(wMsg); // start read from uart_w

    // there are always two pending read requests
    while (true) {
        auto& m = sys::recv();
        assert(m.mTag == 'R');
        assert(m.mDst == console.dId || m.mDst == uart_w.dId);

        uint8_t other = m.mDst ^ console.dId ^ uart_w.dId;
        Message out { other, 'W', m.mLen, m.mPtr };
        sys::call(out);
 
        sys::send(m);
    }
}
