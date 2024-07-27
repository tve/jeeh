// Interface to the 800x480 LCD on the HAOYU STM32F429GI board.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "lcd.h"

int main () {
    initBoard();

    auto sdRam = initSdRam();

    auto& bg = *(lcd::FrameBuffer<1>*) sdRam;
    auto& fg = *(lcd::FrameBuffer<2>*) (&bg+1);
    static_assert(sizeof bg == lcd::HEIGHT * lcd::WIDTH);
    static_assert(sizeof fg == lcd::HEIGHT * lcd::WIDTH);

    logf("sdram %p, bg %p, fg %p", sdRam, &bg, &fg);

    lcd::init();
    bg.init();
    fg.init();

    Pin ledR ("J15");
    ledR.mode("P");
    ledR = 0; // lcd reset
    sys::wait(20);
    ledR = 1;
    sys::wait(10);

    Pin ledB ("I14");
    ledB.mode("P");
    ledB = 1; // lcd backlight

    for (int y = 0; y < lcd::HEIGHT; ++y)
        for (int x = 0; x < lcd::WIDTH; ++x)
            bg(x, y) = x ^ y;

    for (int y = 0; y < lcd::HEIGHT; ++y)
        for (int x = 0; x < lcd::WIDTH; ++x)
            fg(x, y) = ((16*x)/lcd::WIDTH << 4) | (y >> 4);

    while (true) {
        led.toggle();
        sys::wait(500);
    }
}
