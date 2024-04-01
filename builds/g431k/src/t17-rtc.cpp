// Check the built-in real-time clock and backup registers.

#include <jee.h>
using namespace jeeh;
#include "test.h"

void showTime (int n, DateTime const& dt ={}) {
    logf("%d: %02d/%02d/%02d %02d:%02d:%02d = %9u s",
            n, dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, (uint32_t) dt);
}

int main () {
    Tester t;

#if STM32G431xx
    // there are no OSC32 pins on Nucleo-32's G431KB, must use the 32 kHz LSI
    RCC(ena::PWR, 1) = 1;
    PWR[0x00](8) = 1; // DBP
                      //
    RCC[0x90](9) = 1;             // RTSEL = LSI
    RCC[0x90](15) = 1;            // RTCEN
#else    
    rtc::init();
#endif
    rtc::set({ 1, 2, 3, 11, 22, 33 });

    showTime(1, rtc::getDate());

    for (auto i = 0; i < 10; ++i) {
        sys::wait(101);
        itmWrite(".", 1); // send some output to avoid a timeout
    }
    itmWrite("\n", 1);

    showTime(2, rtc::getDate());

    rtc::set({ 12, 11, 10, 23, 22, 21 });
    showTime(3, rtc::getDate());

    for (auto i = 0; i < 10; ++i)
        rtc::setReg(i, i * 1111); // regs are only 16-bit on F1xx
    for (auto i = 0; i < 10; ++i)
        logf("%d: %04d", i, rtc::getReg(i));
}
