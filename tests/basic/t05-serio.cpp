#include <jee.h>
using namespace jeeh;
#include "defs.h"
#include "test.h"

#if STM32F4
enum { ISR=0x00, RDR=0x04, TDR=0x04, BRR=0x08, CR1=0x0C, UE=13 };
#else
enum { CR1=0x00, BRR=0x0C, ISR=0x1C, RDR=0x24, TDR=0x28, UE=0 };
#endif

bool serio () {
    Pin::config(UART_PINS);
    RCC(ena::UART_NAME, 1) = 1;
    UART_NAME[BRR] = SystemCoreClock / 115'200;
    UART_NAME[CR1] = (1<<29) | (1<<3) | (1<<2) | (1<<UE);  // FIFOEN TE RE UE
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

    if (serio()) {
        // msg must fit in the UART's FIFO (i.e. the TX+RX FIFO's combined)
        auto msg = "Hello world!";
        for (auto p = msg; *p != 0; ++p)
            putch(*p);
        for (auto p = msg; *p != 0; ++p) {
            auto c = getch();
            itmWrite(&c, 1);
            itmWrite("\n", 1);
            assert(c == *p);
        }
    } else {
        auto msg = "Howdy!"; // there's no FIFO
        for (auto p = msg; *p != 0; ++p) {
            putch(*p);
            auto c = getch();
            itmWrite(&c, 1);
            itmWrite("\n", 1);
            assert(c == *p);
        }
    }
}
