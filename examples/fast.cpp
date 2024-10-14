#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

static void delayLoop (uint16_t ms) {
    cycles::init();
    while (cycles::count() < ms * (SystemCoreClock/1000)) {}
}

int main () {
    fastClock(); // 160 MHz

    Pin led ("B8","P");

    while (true) {
        led = 1;
        delayLoop(100);
        led = 0;
        delayLoop(400);
    }
}
