// Access various memory blocks on the HAOYU STM32F429GI board.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "memtest.h"

void initFsmcPins () {
    RCC(ena::FMC, 1) = 1;
    Pin::config("B7:V12,C0,C3,"
                "D0,D1,D4,D5,D6,D7,D8,D9,D10,D11,D12,D13,D14,D15,"
                "E0,E1,E2,E3,E4,E5,E7,E8,E9,E10,E11,E12,E13,E14,E15,"
                "F0,F1,F2,F3,F4,F5,F11,F12,F13,F14,F15,"
                "G0,G1,G2,G3,G4,G5,G8,G9,G10,G12,G15,"
                "H3,H5");
}

int main () {
    initBoard();
    printf("%s: mem @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);

    initFsmcPins();

    // setup 4 MB PSRAM
    enum { BCR4=0x18, BTR4=0x1C };
    FMC[BCR4] = (1<<12) | (1<<7) | (1<<4) | (1<<2); // WREN b7 MWID MTYP
    FMC[BTR4] = (1<<16) | (8<<8) | (6<<0); // BUSTURN DATAST ADDSET
    FMC[BCR4](0) = 1; // MBKEN

    auto SRAM = (uint8_t*) 0x6C00'0000;
    for (auto i = 0; i < 64; ++i)
        SRAM[i] = i;
    dumpHex(SRAM, 64, "4 MB PSRAM");
    printf(" %d errors\n", memTests((uint32_t) SRAM, 4<<20));

    while (true) {
        ledL.toggle();  // red, left
        sys::wait(500);
    }
}
