// Examine the control flow of automatic stop mode.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();
    Pin::config("B4:V15"); // EVENTOUT

    RCC[0x08](28,3) = 4;    // 1/16 prescaler
    RCC[0x08](24,4) = 1;    // SYSCLK
    //RCC[0x08](24,4) = 6;    // LSI
    Pin::config("A8:0");    // MCO

    auto t1 = cycles::millis();
    auto t2 = rtc::getDate().todMillis();
    logf("cy %d ms, rtc %d ms", t1, t2);

    while (true) {
        Tracer<10> pt;
        //sys::wait(1000);
        rtc::shortSleep(1000, sys::STOP2);
        logf("%d %d", cycles::millis() - t1, rtc::getDate().todMillis() - t2);
    }
}

uint8_t jeeh::lowestPower (uint8_t power, uint16_t) {
    Tracer<11> pt;
    return sys::STOP0;
}

void jeeh::resumePower () {
    Tracer<12> pt;
}
