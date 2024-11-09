// Capture DCF77 signals at 1 kHz.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

Ticker ticker;
TICKER_INSTALL(ticker)

struct Stream : Worker {
    enum TAG { START, TICK, SENT };

    uint16_t bits, repeat =0, prev =0;
    uint8_t count =0;
    char buf [20];

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(1, TICK);
                break;
            case TICK:
#if 1
                led = +dcfDat;
#else
                led = !msfDat; // inverted logic
#endif
                bits >>= 1;
                bits |= led<<15;
                if (++count >= 16) {
                    count = 0;
                    if (bits == prev && repeat < 99)
                        ++repeat;
                    else {
                        auto n = snprintf(buf, sizeof buf, "%2d %d\n",
                                            repeat, (int16_t) prev);
                        ttyUart.write(buf, n, { wId, SENT });
                        repeat = 1;
                        prev = bits;
                    }
                }
                break;
            case SENT:
                break; // assume that it completes before the next write
            default:
                fail();
        }
        return out;
    }
};

// The output stream consists of text lines with 2 integers:
//      <rept> is a repeat count of the next integer
//      <bits> is a signed 16-bit int, with pulse values in bits 0..15
// A squeaky clean signal could be transmitted as 3 text lines:
//      "50 0", "12 -1", and "1 255" (without the quotes)
// That's 50x16x "0" (800x "0"), 12x16x "1" (192x "1"), 8x "1", and 8x "0".

// Here is an actual capture of a very long "0" series at the minute mark:
//
//  55 0
//   6 -1
//   1 511
//  99 0
//  19 0
//   6 -1
//   1 4095
//  55 0
//
// That's 880x "0", 105x "1", 1895x "0", 107x "1", 880x "0".


int main () {
    initBoard();
    dcfVcc = 1;
    msfVcc = 1;

    Stream stream;
    ticker.init();
    stream.init();

    while (true)
        asm ("wfi");
}
