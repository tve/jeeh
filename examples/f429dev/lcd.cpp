// Access PSRAM and SDRAM memory on the HAOYU STM32F429GI board.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "memtest.h"

void initLcdPins () {
    RCC(ena::FMC, 1) = 1;
    Pin::config("F10:V14,G6,G7,G11,"
                "H9,H10,H11,H12,H13,H14,H15,"
                "I0,I1,I2,I4,I5,I6,I7,I9,I10");
    ledB = 1; // backlight
}

namespace lcd {

enum { WIDTH=800,HEIGHT=480,HSYN=48,HBP=88,HFP=40,VSYN=3,VBP=32,VFP=13 };

// shorthand to set both horizontal and vertical parameters
constexpr auto hv (uint16_t h, uint16_t l) { return (h << 16) | l; }

void init () {
    RCC(ena::LTDC, 1) = 1;
    RCC[0x88] = (3<<28) | (7<<24) | (192<<6); // PLLSAICFGR
    RCC[0x8C] = 0;

    RCC[0x00](28) = 1;            // PLLSAION in CR
    while (RCC[0x00](29) == 0) {} // wait for PLLSAIRDY in CR

    LTDC[0x08] = hv(HSYN-1, VSYN-1);                              // SSCR
    LTDC[0x0C] = hv(HSYN+HBP-1, VSYN+VBP-1);                      // BPCR
    LTDC[0x10] = hv(HSYN+HBP+WIDTH-1, VSYN+VBP+HEIGHT-1);         // AWCR
    LTDC[0x14] = hv(HSYN+HBP+WIDTH+HFP-1, VSYN+VBP+HEIGHT+VFP-1); // TWCR

    LTDC[0x18] = (1<<16) | (1<<0); // DEN & LTDCEN in GCR
    LTDC[0x24](0) = 1;             // IMR in SRCR
}

template <int N>
struct FrameBuffer {
    constexpr static IoReg<LTDC.ADDR+0x80*N> LAYER {};
    static_assert(N == 1 || N == 2);

    void init () {
        LAYER[0x04] = 0;                               // ~LEN in LxCR
        LAYER[0x08] = hv(HSYN+HBP+WIDTH-1, HSYN+HBP);  // LxWHPCR
        LAYER[0x0C] = hv(VSYN+VBP+HEIGHT-1, VSYN+VBP); // LxWVPCR
        LAYER[0x10] = 0;                               // LxCKCR
        LAYER[0x14] = N == 1 ? 0b101 : 0b110;          // LxPFCR L8/AL44
        LAYER[0x2C] = (uint32_t) data;                 // LxCFBAR
        LAYER[0x30] = hv(WIDTH, WIDTH+3);              // LxCFBLR
        LAYER[0x34] = HEIGHT;                          // LxCFBLNR

        if constexpr (N == 1)
            for (int i = 0; i < 256; ++i) {
                auto r = i>>5, g = (i>>2) & 7, b = i & 3;
                auto rgb = (r<<21)|(r<<18)|(g<<13)|(g<<10)|(b<<6)|(b<<4)|(b<<2);
                // rgb = (i << 16) | (i << 8) | (i << 0); // greyscale
                LAYER[0x44] = (i<<24) | rgb;
            }
        else
            for (int i = 0; i < 16; ++i) {
                uint8_t r = -((i>>2)&1), g = -((i>>1)&1), b = -(i&1);
                if (i < 8) {
                    r >>= 2; g >>= 2; b >>= 2;
                }
                LAYER[0x44] = (i<<24) | (r<<16) | (g<<8) | b;
            }

        LAYER[0x04] = (1<<4) | (1<<0); // CLUTEN & LEN in LxCR
        LAYER[0x1C] = 0;               // LxDCCR transparent

        LTDC[0x24](0) = 1; // IMR in SRCR
    }

    auto& operator() (int x, int y) { return data[y][x]; }

    uint8_t data [HEIGHT][WIDTH];
};

} // namespace lcd

int main () {
    initBoard();
    printf("%s: lcd @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);

    initFmcPins();
    auto psRam = initPsRam();
    auto sdRam = initSdRam();

    auto& bg = *(lcd::FrameBuffer<1>*) sdRam;
    auto& fg = *(lcd::FrameBuffer<2>*) (&bg+1);
    static_assert(sizeof bg == lcd::HEIGHT * lcd::WIDTH);
    static_assert(sizeof bg == lcd::HEIGHT * lcd::WIDTH);

    printf("psram %p, sdram %p, bg %p, fg %p\n", psRam, sdRam, &bg, &fg);

    initLcdPins();
    lcd::init();
    bg.init();
    fg.init();

    for (int y = 0; y < lcd::HEIGHT; ++y)
        for (int x = 0; x < lcd::WIDTH; ++x)
            bg(x, y) = x ^ y;

    for (int y = 0; y < lcd::HEIGHT; ++y)
        for (int x = 0; x < lcd::WIDTH; ++x)
            fg(x, y) = ((16*x)/lcd::WIDTH << 4) | (y >> 4);

    while (true) {
        ledL.toggle();
        sys::wait(500);
    }
}
