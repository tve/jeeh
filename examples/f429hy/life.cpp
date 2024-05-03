// Conway's "Game of Life" on a 400x240 grid.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "lcd.h"

int main () {
    initBoard();
    printf("%s: lcd @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);

    initFmcPins();
    auto sdRam = initSdRam();

    auto& bg = *(lcd::FrameBuffer<1>*) sdRam;
    static_assert(sizeof bg == lcd::HEIGHT * lcd::WIDTH);

    lcd::init();
    bg.init();
    ledB = 1; // backlight
    
    constexpr auto X = 400, Y = 240;
    static uint8_t g [X][Y];

    auto r = 0;
    while (true) {
        memset(g, 0, sizeof g);
        for (int y = 1; y < Y-1; ++y)
            for (int x = 1; x < X-1; ++x)
                g[x][y] = ((x^r)*y & 0xC0) == 0;

        printf("%d\n", r++);

        for (auto k = 0; k < 1000; ++k) {
            ledL.toggle();

            for (int y = 1; y < Y-1; ++y)
                for (int x = 1; x < X-1; ++x)
                    g[x][y] <<= 4;

            for (int y = 1; y < Y-1; ++y)
                for (int x = 1; x < X-1; ++x) {
                    auto c = g[x-1][y-1] + g[x+0][y-1] + g[x+1][y-1] +
                        g[x-1][y+0]               + g[x+1][y+0] +
                        g[x-1][y+1] + g[x+0][y+1] + g[x+1][y+1];
                    switch (c >> 4) {
                        case 2: if (!(g[x][y] >> 4))
                                    break;
                                [[fallthrough]];
                        case 3: g[x][y] += 1;
                    }
                }

            for (int y = 0; y < Y; ++y)
                for (int x = 0; x < X; ++x)
                    bg(2*x+0, 2*y+0) =
                        bg(2*x+1, 2*y+0) =
                        bg(2*x+0, 2*y+1) =
                        bg(2*x+1, 2*y+1) = -(g[x][y] & 1);
        }
    }
}
