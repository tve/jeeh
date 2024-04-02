// Use the watchdog to reset the system after a while.

#include <jee.h>
using namespace jeeh;
#include "test.h"

void showTime (int n, DateTime const& dt ={}) {
    logf("%d: %02d/%02d/%02d %02d:%02d:%02d = %9u s",
            n, dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (uint32_t) dt);
}

int main () {
    Tester t;

#if 0
    logf("compiled on <%s> at <%s>\n", __DATE__, __TIME__);
    showTime(0);
#endif

    showTime(1, { 1, 2, 3, 11, 22, 33 });
    showTime(2, DateTime (34514553));
    showTime(3, DateTime ((uint32_t) 0));
    showTime(4, DateTime (86400));
    showTime(5, DateTime (31*86400));
    showTime(6, DateTime (366*86400));
}
