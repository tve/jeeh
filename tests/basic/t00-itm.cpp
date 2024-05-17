#include <jee.h>
using namespace jeeh;

void jeeh::fail (void const*, char const*, int) {
    while (true) {}
}

[[gnu::weak]] void LowPower::start (Message&) {}
[[gnu::weak]] void LowPower::finish () {}

int main () {
    fastClock();
#if SWO_FREQ
    swoInit(SWO_FREQ); // TODO openocd didn't init ITM/SWO on STM32WL
#endif
    itmWrite("TEST\n"
             "OK\n", 8);
}
