// Try out Segger's RTT mechanism.

#include <jee.h>
#include "defs.h"
using namespace jeeh;

char txBuf [4096] alignas (4), rxBuf [16] alignas (4);

struct RttTX {
    char const* name;
    char* buf;
    uint32_t len;
    uint32_t in;
    volatile uint32_t out;
    uint32_t flags;
};

struct RttRX {
    char const* name;
    char* buf;
    uint32_t len;
    volatile uint32_t in;
    uint32_t out;
    uint32_t flags;
};

struct RttDesc {
    enum { NTX=1, NRX=1 };

    char id [16] = "xEGGER RTT"; // intentional mismatch, replace x w/ S
    int nTx =NTX;
    int nRx =NRX;
    RttTX tx [NTX] = {{ "Terminal", txBuf, sizeof txBuf, 0, 0, 0 }};
    RttRX rx [NRX] = {{ "Terminal", rxBuf, sizeof rxBuf, 0, 0, 0 }};
    uint8_t pad [24];
};

RttDesc _SEGGER_RTT alignas (32);
RttDesc* rttPtr;

constexpr Pin led (LED);

void rttWrite (char const* ptr, int len) {
    auto& tx = _SEGGER_RTT.tx[0];
    auto curr = tx.in;
//led = 0;
    for (auto i = 0; i < len; ++i) {
        tx.buf[curr] = *ptr++;
        auto next = (curr + 1) % tx.len;
        //while (next == tx.out) {} // busy loop
        curr = next;
        if (curr % 64 == 0)
            tx.in = curr; // "flush" into buffer
    }
//led = 1;
    tx.in = curr;
}

int main () {
    //fastClock();
    _SEGGER_RTT.id[0] = 'S'; // fix the unique tag so the debugger finds it
    led.mode("P");

    char c = '@';
    while (true) {
        led.toggle();
        sys::wait(100);

        rttWrite (&c, 1);
        if (++c > '~') {
            c = '@';
            rttWrite ("\n", 1);
        }
    }
}
