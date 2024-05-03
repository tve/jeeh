// Conway's "Game of Life" on a 400x240 grid.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "lcd.h"

int main () {
    initBoard();
    auto mhz = SystemCoreClock / 1'000'000;
    printf("%s: lcd @ %d MHz\n", SVDNAME, mhz);

    initFmcPins();
    auto sdRam = initSdRam();

    auto& bg = *(lcd::FrameBuffer<1>*) sdRam;
    static_assert(sizeof bg == lcd::HEIGHT * lcd::WIDTH);

    lcd::init();
    bg.init();
    ledB = 1; // backlight
    
    constexpr auto X = 400, Y = 240;
    static uint8_t g [Y][X];
    auto p = (uint16_t*) bg.data;

    auto r = 0;
    while (true) {
        memset(g, 0, sizeof g);
        for (int y = 1; y < Y-1; ++y)
            for (int x = 1; x < X-1; ++x)
                g[y][x] = ((x^r)*y & 0xC0) == 0;

        printf("%d\n", r++);

        for (auto k = 0; k < 1000; ++k) {
            ledL.toggle();

            cycles::init();
            for (int y = 1; y < Y-1; ++y)
                for (int x = 1; x < X-1; ++x)
                    g[y][x] <<= 4;
            auto t1 = cycles::count();

            cycles::init();
            for (int y = 1; y < Y-1; ++y)
                for (int x = 1; x < X-1; ++x) {
                    auto c = g[y-1][x-1] + g[y-1][x] + g[y-1][x+1] +
                             g[y  ][x-1]             + g[y  ][x+1] +
                             g[y+1][x-1] + g[y+1][x] + g[y+1][x+1];
                    switch (c >> 4) {
                        case 2: if (!(g[y][x] >> 4))
                                    break;
                                [[fallthrough]];
                        case 3: g[y][x] += 1;
                    }
                }
            auto t2 = cycles::count();

            cycles::init();
            for (int y = 1; y < Y-1; ++y)
                for (int x = 1; x < X-1; ++x)
                    p[2*y*X+x] = p[(2*y+1)*X+x] = -(g[y][x] & 1);
            auto t3 = cycles::count();

            if (k % 100 == 0) {
                auto fps = (10*SystemCoreClock)/(t1+t2+t3);
                printf("  %d us, %d us %d us, %d.%d fps\n",
                        t1/mhz, t2/mhz, t3/mhz, fps/10, fps%10);
            }
        }
    }
}
