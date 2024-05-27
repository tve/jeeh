// Serial port output test.

#include <jee.h>
#include "defs.h"
using namespace jeeh;

enum { CR1=0x00, BRR=0x0C, ISR=0x1C, RDR=0x24, TDR=0x28, UE=0 };

void serio () {
    Pin::config(UART_PINS);
    RCC(ena::UART_NAME, 1) = 1;
    UART_NAME[BRR] = 16; // 16 MHz => 1 Mbd
    UART_NAME[CR1] = (1<<3) | (1<<2) | (1<<UE);  // TE RE UE
}

void putch (char c) {
    while (!UART_NAME[ISR](7)) {} // TXE
    UART_NAME[TDR] = c;
    while (UART_NAME[ISR](6) == 0) {} // TC
}

int getch () {
    while (!UART_NAME[ISR](5)) {} // RXFNE
    return UART_NAME[RDR];
}
int main () {
    fastClock();

    //rtc::init(false);
    serio();
    RCC[0x88](2, 2) = 2; // HSI clock for USART2

    Pin pa0 ("A0"); pa0.mode("P");
    Pin pa1 ("A1"); pa1.mode("P");
    pa0 = 1;
    sys::wait(3);

    for (auto i = 0; i < 1000; ++i) {
        putch('0' + i % 10);
        pa1.toggle();
        //sys::wait(10);
        rtc::deepSleep(20, 0);
    }
}
