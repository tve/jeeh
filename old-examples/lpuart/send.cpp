// Try out the Low-Power UART.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    auto id = loopback.dId;
    //auto id = lowpower.dId;

    Message r { id, 'R' };
    int seq = 0;
    while (true) {
        uint8_t buf [50];
        auto n = snprintf((char*) buf, sizeof buf,
                    "Hello, loopback greetings #%d !", ++seq);
        Message w { id, 'W', (uint16_t) n, buf };

        cycles::clear();
        sys::call(w);
        auto t = cycles::count();
        printf("%6d cycles: ", t);

        do {
            sys::call(r);
            assert(r.mLen > 0);

            memcpy(buf, r.mPtr, r.mLen);
            buf[r.mLen] = 0;
            printf("<%s>", buf);
        } while (buf[r.mLen-1] != '!');
        printf("\n");

        led.toggle();
        sys::wait(500);
    }
}
