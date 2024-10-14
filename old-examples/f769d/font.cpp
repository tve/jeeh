// Interface to the 800x480 LCD on the HAOYU STM32F429GI board.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "lcd.h"

static uint8_t font [95][16] = {
#include "font.h"
};

int main () {
    initBoard();

    auto sdRam = initSdRam();

    auto& bg = *(lcd::FrameBuffer<1>*) sdRam;
    static_assert(sizeof bg == lcd::HEIGHT * lcd::WIDTH);

    logf("sdram %p, bg %p, fg %p", sdRam, &bg, &fg);

    lcd::init();
    bg.init();
    ledB = 1; // backlight

    uint32_t expand [16];
    for (auto i = 0; i < 16; ++i) {
        expand[i] = 0;
        for (auto j = 0; j < 4; ++j)
            if ((i << j) & 0x08)
                expand[i] |= 0xFF << (8*j);
    }

#if 1
    for (int x = 0; x < lcd::WIDTH/8; ++x)
        for (int y = 0; y < lcd::HEIGHT/16; ++y) {
            auto v = (x + y) % 95;
            for (int r = 0; r < 16; ++r) {
#if 0 // dumb loop:                      30.7 ms for 100x30 chars = 10.2 µs/ch
                for (int c = 0; c < 8; ++c) {
                    auto z = (font[v][r] << c) & 0x80 ? 0xFF : 0x00;
                    bg(8*x+c, 16*y+r) = z;
                }
#else // expand in loop:                   8.2 ms for 100x30 chars = 2.7 µs/ch
                auto f = font[v][r];
                auto p = (uint32_t*) &bg(8*x, 16*y+r);
                p[0] = expand[f>>4];
                p[1] = expand[f&0x0F];
#endif
            }
        }
#else // top to bottom loop:               9.9 ms for 100x30 chars = 3.3 µs/ch
    auto p = (uint32_t*) bg.data;
    for (int y = 0; y < lcd::HEIGHT/16; ++y)
        for (int r = 0; r < 16; ++r)
            for (int x = 0; x < lcd::WIDTH/8; ++x) {
                auto v = (x + y) % 95;
                auto f = font[v][r];
                *p++ = expand[f>>4];
                *p++ = expand[f&0x0F];
            }
#endif

    logf("done, %d us", cycles::count() / (SystemCoreClock/1'000'000));
    while (true) {
        ledL.toggle();
        sys::wait(500);
    }
}
