#include <jee.h>
using namespace jeeh;

void jeeh::fail (void const*, char const*, int) {
    while (true) {}
}

int main () {
    fastClock();
#if SWO_FREQ
    swoInit(SWO_FREQ); // TODO openocd didn't init ITM/SWO on STM32WL
#endif
    swoWrite("TEST\n"
             "OK\n", 8);
}
