#include <jee.h>
using namespace jeeh;

// avoid pulling in logf, etc
void jeeh::fail (void const*, char const*, int) { while (true) {} }
void jeeh::hardFaultHandler (uint32_t*) { fail(); }

int main () {
    fastClock();
#if SWO_FREQ
    swoInit(SWO_FREQ); // TODO openocd didn't init ITM/SWO on STM32WL
#endif
    swoWrite("TEST\n"
             "OK\n", 8);
}
