#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;

int main () {
    fastClock(); // 160 MHz

    Pin led ("B8","P");

    while (true) {
        led = 1;
        cycles::msBusy(100);
        led = 0;
        cycles::msBusy(400);
    }
}
