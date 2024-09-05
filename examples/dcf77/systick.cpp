// Decode DCF77 using a convolution kernel, called from the SysTick IRQ.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "decoder.h"

Decoder d;

// this needs "-DMYSYSTICK" to disable JeeH's default SysTick handler
extern "C" void SysTick_Handler () {
    led = dcfData;
    d.step(led);
}

int main() {
    initBoard();

    STK[0x4] = (SystemCoreClock/256) / 8 - 1; // reload value for ≈256 Hz
    STK[0x8] = 0;
    STK[0x0] = 0b011; // enable, clk/8 mode

    while (true)
        asm ("wfi");
}
