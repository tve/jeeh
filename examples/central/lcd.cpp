// Demo of the on-board LCD.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#include "lcd-st7789.h"

ST7789 lcd;

const auto lcdAddr = (uint16_t*) 0x6400'0000;

void ST7789::cmd (int v) { asm ("dmb"); lcdAddr[0] = v; }
void ST7789::dat (int v) { asm ("dmb"); lcdAddr[1] = v; }

void initLcd () {
    enum { BCR2=0x08, BTR2=0x0C };

    FMC[BCR2] = (1<<12) | (1<<7) | (1<<4);
    FMC[BTR2] = (1<<20) | (6<<8) | (2<<4) | (9<<0);
    FMC[BCR2](0) = 1; // MBKEN

    Pin reset ("H7");
    reset.mode("P");
    sys::wait(5); reset = 1; sys::wait(5);

    lcd.init();
    lcd.clear();

    Pin light ("H11");
    light.mode("P");
    light = 1; // on
}

void lcdCmd () {
}    

int main () {
    initBoard(); // in defs.h
    initFsmcPins();

    initLcd();

    printf("%dx%d @ 0x%08x\n", lcd.width, lcd.height, lcdAddr);
    initLcd();

    auto t = cycles::micros();
    auto n = cycles::count();

    lcd.clear();

    n = cycles::count() - n;
    t = cycles::micros() - t;
    printf("  clear: %7d cycles, %5d us\n", n, t);

    t = cycles::micros();
    n = cycles::count();

    lcd.fill(50, 10, 100, 100, 0xF800); // red
    lcd.fill(20, 200, 200, 10, 0x07E0); // green
    lcd.fill(200, 60, 30, 100, 0x001F); // blue

    n = cycles::count() - n;
    t = cycles::micros() - t;
    printf("  fills: %7d cycles, %5d us\n", n, t);
    
    while (true) { led.toggle(); sys::wait(250); }
}
