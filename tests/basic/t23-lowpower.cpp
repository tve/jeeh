#include <jee.h>
using namespace jeeh;
#define OWN_LOWPOWER 1
#include "test.h"

void LowPower::start (Message& m) {
    m.mTag = m.mLen >= 50 ? Device::STOP1 :
             m.mLen >= 30 ? Device::STOP0 :
                            Device::SLOWEST;
    logf("S %d -> %d", m.mLen, m.mTag);
    for (auto i = 0; i < 500; ++i) asm (""); // let the ITM/SWO logs drain
}

void LowPower::finish () {
    fastClock();
    logf("F");
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

    constexpr int delays [] = { 20, 30, 40, 50, 60, 40, 20 };
    for (auto ms : delays) {
        auto t1 = rtc::todMillis();
        sys::wait(ms);
        auto t2 = rtc::todMillis();

        int n = t2 - t1;
        logf("%d ms: %d", ms, n);
        assert(ms-5 < n && n < ms+5);
    }
}
