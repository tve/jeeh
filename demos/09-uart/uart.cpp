#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

uart::Poll<USART2.ADDR> gps (ena::USART2, 50);

int main () {
    initBoard();
    gps.init("A2:7,A3", 9600);

    while (true) {
        uint8_t ch;
        gps.transfer(false, &ch, 1);
        serio::write(&ch, 1);
    }
}
