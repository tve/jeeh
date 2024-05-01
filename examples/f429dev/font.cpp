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
    printf("%s: lcd @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);

    initFmcPins();
    auto sdRam = initSdRam();

    auto& bg = *(lcd::FrameBuffer<1>*) sdRam;
    static_assert(sizeof bg == lcd::HEIGHT * lcd::WIDTH);

    printf("sdram %p, bg %p\n", sdRam, &bg);

    lcd::init();
    bg.init();
    ledB = 1; // backlight
    cycles::init();

    for (int x = 0; x < lcd::WIDTH/8; ++x)
        for (int y = 0; y < lcd::HEIGHT/16; ++y) {
            auto v = (x + y) % 95;
            for (int r = 0; r < 16; ++r)
                for (int c = 0; c < 8; ++c) {
                    auto z = (font[v][r] << c) & 0x80 ? 0xFF : 0x00;
                    bg(8*x+c, 16*y+r) = z;
                }
        }

    printf("done, %d us\n", cycles::count() / (SystemCoreClock/1'000'000));
    while (true) {
        ledL.toggle();
        sys::wait(500);
    }
}
