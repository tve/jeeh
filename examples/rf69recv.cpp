// Show incoming wireless RF69 packets on USART1.

#include <jee.h>
#include <jee/spi-rf69.h>

UartBufDev< PinA<9>, PinA<10> > uart;  // use buffering to avoid missed packets

void printf(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); veprintf(uart.putc, fmt, ap); va_end(ap);
}

RF69< PinA<7>, PinA<6>, PinA<5>, PinA<4> > rf;  // default SPI1 pins

PinC<13> led;

int main () {
    led.mode(Pinmode::out);
    rf.init(1, 6, 8686);

    while (true) {
        uint8_t rxBuf [64];
        auto rxLen = rf.receive(rxBuf, sizeof rxBuf);

        if (rxLen >= 0) {
            led.toggle();

            printf("RF69 #%d: ", rxLen);
            for (int i = 0; i < rxLen; ++i)
                printf("%02x", rxBuf[i]);
            printf("\n");
        }
    }
}
