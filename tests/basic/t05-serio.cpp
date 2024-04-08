#include <jee.h>
using namespace jeeh;
#include "defs.h"
#include "test.h"

enum { CR1=0x00, BRR=0x0C, ISR=0x1C, RDR=0x24, TDR=0x28 };

bool serio () {
    Pin::config(UART_PINS);
    RCC(ena::UART_NAME, 1) = 1;
    UART_NAME[BRR] = SystemCoreClock / 115'200;
    UART_NAME[CR1] = (1<<29) | (1<<3) | (1<<2) | (1<<0);  // FIFOEN TE RE UE
    return UART_NAME[CR1](29); // only true if the FIFO flag "sticks"
}

void putch (char c) {
    while (!UART_NAME[ISR](7)) {} // TXFNF
    UART_NAME[TDR] = c;
}

int getch () {
    while (!UART_NAME[ISR](5)) {} // RXFNE
    return UART_NAME[RDR];
}

int main () {
    Tester t;

    // this has to fit in the UART's FIFO (i.e. the TX+RX FIFO's combined)
    auto msg = "Hello world!";

    if (serio()) {
        for (auto p = msg; *p != 0; ++p)
            putch(*p);
        for (auto p = msg; *p != 0; ++p) {
            auto c = getch();
            itmWrite(&c, 1);
            itmWrite("\n", 1);
            assert(c == *p);
        }
    } else {
        // there's no FIFO
        for (auto p = msg; *p != 0; ++p) {
            putch(*p);
            auto c = getch();
            itmWrite(&c, 1);
            itmWrite("\n", 1);
            assert(c == *p);
        }
    }
}
