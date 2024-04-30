// Access various memory blocks on the HAOYU STM32F429GI board.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "memtest.h"

void initFmcPins () {
    RCC(ena::FMC, 1) = 1;
    Pin::config("B7:V12,C0,C3,"
                "D0,D1,D4,D5,D6,D7,D8,D9,D10,D11,D12,D13,D14,D15,"
                "E0,E1,E2,E3,E4,E5,E7,E8,E9,E10,E11,E12,E13,E14,E15,"
                "F0,F1,F2,F3,F4,F5,F11,F12,F13,F14,F15,"
                "G0,G1,G2,G3,G4,G5,G8,G9,G10,G12,G15,"
                "H2,H3,H5");
}

int main () {
    initBoard();
    printf("%s: mem @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);

    initFmcPins();

    // set up 4 MB PSRAM
    enum { BCR4=0x18, BTR4=0x1C };
    FMC[BCR4] = (1<<12) | (1<<7) | (1<<4) | (1<<2); // WREN b7 MWID MTYP
    FMC[BTR4] = (1<<16) | (8<<8) | (6<<0); // BUSTURN DATAST ADDSET
    FMC[BCR4](0) = 1; // MBKEN

    auto SRAM = (uint8_t*) 0x6C00'0000;
    for (auto i = 0; i < 64; ++i)
        SRAM[i] = i;
    dumpHex(SRAM, 64, "4 MB PSRAM");
    printf(" %d errors\n", memTests((uint32_t) SRAM, 4<<20));

    // set up 32 MB SDRAM
    enum {CR1=0x140,CR2=0x144,TR1=0x148,TR2=0x14C,CMR=0x150,RTR=0x154,SR=0x158};
    FMC[CR1] = (1<<13)|(1<<12)|(2<<10)|(3<<7)|(1<<6)|(1<<4)|(2<<2)|(1<<0);
    FMC[TR1] = (1<<24)|(1<<20)|(1<<16)|(5<<12)|(3<<8)|(6<<4)|(1<<0);

    // SDRAM commands
    auto fmcWait = []() { while (FMC[SR](5)) {} };
    fmcWait(); FMC[CMR] = (1<<4)|(1<<0); sys::wait(10); // clock enable
    fmcWait(); FMC[CMR] = (1<<4)|(2<<0);                // precharge
    fmcWait(); FMC[CMR] = (1<<4)|(3<<0);                // auto-refresh
    fmcWait(); FMC[CMR] = (0x231<<9)|(1<<4)|(4<<0);     // load mode
    fmcWait(); FMC[RTR] = (51<<1);                      // refresh rate

    auto SDRAM = (uint8_t*) 0xC000'0000;
    for (auto i = 0; i < 64; ++i)
        SDRAM[i] = i+0x40;
    dumpHex(SDRAM, 64, "32 MB SDRAM");
    printf(" %d errors\n", memTests((uint32_t) SDRAM, 32<<20));

    while (true) {
        ledL.toggle();  // red, left
        sys::wait(500);
    }
}
