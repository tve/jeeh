#include <jee.h>
using namespace jeeh;
#include "test.h"

uint8_t jeeh::lowestPower (uint8_t power, uint16_t) {
    return power >= 50 ? sys::STOP1 :
           power >= 40 ? sys::STOP0 :
                         sys::SLOWEST;
}

void jeeh::resumePower () {
    fastClock();
}

int main () {
    Tester t;

    rtc::init(false); // no 32 kHz xtal on Nucleo-G431KB
    rtc::set({ 1, 2, 3, 11, 22, 33 });

    logf("wait 10"); swoWrite();
    sys::wait(10);

    logf("send 15"); swoWrite();
    Message m { '@', 'T', 15 };
    sys::send(m);

    logf("recv"); swoWrite();
    sys::recv();

    constexpr int delays [] = { 30, 40, 50, 100, 200 };
    for (auto ms : delays) {
        auto t1 = rtc::getDate();
        sys::wait(ms);
        auto t2 = rtc::getDate();

        int n = t2.todMillis() - t1.todMillis();
        logf("%d ms: %d", ms, n); swoWrite();
        sys::wait(2);
        assert(9*ms <= 10*n && 10*n <= 11*ms); // +/- 10% TODO g431 uses LSI
    }

    fastClock();
}
