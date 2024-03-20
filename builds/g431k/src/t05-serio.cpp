#include <jee.h>
using namespace jeeh;
#include "defs.h"
#include "test.h"

enum { CR1=0x00, BRR=0x0C, ISR=0x1C, RDR=0x24, TDR=0x28 };

void serio () {
    Pin::config(SERIO_PINS);
    RCC(ena::USART1, 1) = 1;
    USART1[BRR] = SystemCoreClock / 115'200;
    USART1[CR1] = (1<<29) | (1<<3) | (1<<2) | (1<<0);  // FIFOEN TE RE UE
}

void putch (char c) {
    while (!USART1[ISR](7)) {} // TXFNF
    USART1[TDR] = c;
}

int getch () {
    while (!USART1[ISR](5)) {} // RXFNE
    return USART1[RDR];
}

int main () {
    Tester t;

    // this has to fit in the UART's FIFO (i.e. the TX+RX FIFO's combined)
    auto msg = "Hello world!";

    serio();
    for (auto p = msg; *p != 0; ++p)
        putch(*p);
    for (auto p = msg; *p != 0; ++p) {
        auto c = getch();
        itmWrite(&c, 1);
        itmWrite("\n", 1);
        assert(c == *p);
    }
}
