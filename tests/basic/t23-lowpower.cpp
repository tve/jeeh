#include <jee.h>
using namespace jeeh;
#define OWN_LOWPOWER 1
#include "test.h"

void LowPower::start (Message& m) {
    m.mTag = m.mLen >= 50 ? Device::STOP1 :
             m.mLen >= 40 ? Device::STOP0 :
                            Device::SLOWEST;
    //logf("S %d -> %d", m.mLen, m.mTag);
}

void LowPower::finish () {
    fastClock();
    //logf("F");
}

int main () {
    Tester t;

    rtc::init(false); // no 32 kHz xtal on Nucleo-G431KB
    rtc::set({ 1, 2, 3, 11, 22, 33 });

    logf("wait 10");
    sys::wait(10);

    logf("send 15");
    Message m { '@', 'T', 15 };
    sys::send(m);

    logf("recv");
    sys::recv();

    constexpr int delays [] = { 30, 40, 50, 100, 200 };
    for (auto ms : delays) {
        itmFlush();
        auto t1 = rtc::towMillis();
        sys::wait(ms);
        auto t2 = rtc::towMillis();

        int n = t2 - t1;
        logf("%d ms: %d", ms, n);
        assert(9*ms <= 10*n && 10*n <= 11*ms); // +/- 10% TODO g431 uses LSI
    }
}
