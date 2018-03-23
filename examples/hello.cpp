#include <jee.h>

UartDev<Pin<'A',9>,Pin<'A',10>> uart;

int main () {
    // TODO adjustable baud rate, it's fixed at 115200 baud @ 8 MHz for now

    while (true) {
        uart.puts("Hello!\n");

        for (int i = 0; i < 1000000; ++i)
            __asm("");
    }
}
