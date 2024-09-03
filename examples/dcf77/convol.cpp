// Decode DCF77 using a convolution kernel (ported from an older project)

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "decoder.h"

int main() {
    initBoard();

    Decoder d;

    int ssr = 0;
    while (true) {
        // wait until the next 256 Hz tick
        while (ssr == RTC[0x08]) {} // sub-second register
        ssr = RTC[0x08];

        led = dcfData;
        d.step(led);
    }
}
