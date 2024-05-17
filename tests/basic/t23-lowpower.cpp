#include <jee.h>
using namespace jeeh;
#define OWN_LOWPOWER 1
#include "test.h"

void LowPower::start (Message& m) {
    //logf("S %d", m.mLen);
    for (auto i = 0; i < 500; ++i) asm (""); // let the ITM/SWO logs drain

    m.mTag = m.mLen >= 40 ? Device::STOP1 :
             m.mLen >= 30 ? Device::STOP0 :
                            Device::SLOWEST;
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

    constexpr int delays [] = { 20, 30, 40, 50, 40, 30, 20 };
    for (auto ms : delays) {
        auto t1 = rtc::todMillis();
        sys::wait(ms);
        auto t2 = rtc::todMillis();
        logf("%d ms: %d", ms, t2 - t1);
    }
}
