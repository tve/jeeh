// Use sys::coma to go into long-term sleep modes.

#include <jee.h>
#include <jee/hal.h>
#include <jee/dev/rf69.h>
using namespace jeeh;
#include "defs.h"

int main () {
    //slowClock(false);

    spi::Gpio rfmBus;
    rfmBus.init(SPI_PINS, 10'000);
for (auto i = 1<<19; i > 0; --i) asm ("");
    RF69 rf (rfmBus);
    rf.init(63, 42, 8686); // node 63, group 42, 868.6 MHz
    rf.sleep();
    rfmBus.deinit();
    //rfmBus.nsel.mode("U");

    //Pin::config("A1:F,A13,A14");
    GPIOA[0x00] = 0xFFFF'FFFF; // MODER all analog
    GPIOB[0x00] = 0xFFFF'FFFF; // MODER all analog
    GPIOC[0x00] = 0xFFFF'FFFF; // MODER all analog

    RCC(ena::GPIOA,1) = 0;
    RCC(ena::GPIOA+1,1) = 0;
    RCC(ena::GPIOA+2,1) = 0;

    rtc::init();
    sys::coma(3, sys::STANDBY);

    led.mode("P");
    while (true) {
        led.toggle();
        sys::wait(100);
    }
}
