// Drive a 1.44" 128x128 LCD display over SPI.

#include <jee/util/twodee.h>
using namespace jeeh::twodee;

// from Oli Kraus' nice font collection, https://github.com/olikraus/u8g2/wiki
#define U8G2_FONT_SECTION(x)
#include "../g474r/font.h"
#include "../g474r/font2.h"
#include "../g474r/font3.h"
Font const font (u8g2_font_lubR08_tr);
Font const font2 (u8g2_font_7x14_tf);
#if !STM32F3 // FIXME ???
Font const font3 (u8g2_font_9x15_tf);
#endif

auto& lcd = spiBus;

Pin& lcdCmd = lcd.miso; // re-used as C/D output pin
//Pin lcdRst {"D15","P"}; // tied to Vcc instead

namespace {

void out16 (uint16_t v) {
    uint8_t hdr [] = { (uint8_t) (v>>8), (uint8_t) v };
    lcd.ioRequest(IO_WRITE, hdr, sizeof hdr);
}

// returns with cs low
void cmd (uint8_t v) {
    lcdCmd = 0;
    lcd.ioRequest(IO_START|IO_WRITE, &v, 1);
    lcdCmd = 1;
}

void cmdEnd () {
    lcd.ioRequest(IO_STOP);
}

void init () {
    static uint8_t const config [] = {
        // cmd, count, data bytes ...
      //0x01,  0,       // Soft reset
      //0xFF,  120,     // reset delay
        0x3A,  1, 0x05, // Set pixel format
#if 0
        0x26,  1, 0x04, // Set Gamma curve 3
        0xF2,  1, 0x01, // Gamma adjustment enabled
        0xE0, 15, 0x3F, 0x25, 0x1C, 0x1E, 0x20, 0x12, 0x2A, 0x90,
                  0x24, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, // Pos Gamma
        0xE1, 15, 0x20, 0x20, 0x20, 0x20, 0x05, 0x00, 0x15,0xA7,
                  0x3D, 0x18, 0x25, 0x2A, 0x2B, 0x2B, 0x3A, // Neg Gamma
#endif
        0xB1,  2, 0x08, 0x08, // Frame rate control 1
        0xB4,  1, 0x07,       // Display inversion
        0xC0,  2, 0x0A, 0x02, // Power control 1
        0xC1,  1, 0x02,       // Power control 2
        0xC5,  2, 0x50, 0x5B, // Vcom control 1
        0xC7,  1, 0x40,       // Vcom offset
        0x36,  1, 0xC8,       // Set address mode

        0x11,  0,       // Exit sleep mode
        0x29,  0,       // Display on
    };

    for (uint8_t const* p = config; p < config + sizeof config; ++p) {
        if (*p == 0xFF)
            cycles::msBusy(*++p);
        else {
            cmd(*p);
            auto n = *++p;
            lcd.ioRequest(IO_WRITE, (uint8_t*) p+1, n);
            p += n;
        }
    }
    cmdEnd();
}

} // namespace

constexpr auto width = 128, height = 128;

uint16_t xLimit = width-1;
uint16_t yLimit = height-1;

void bounds (int xend =width-1, int yend =height-1) {
    xLimit = xend;
    yLimit = yend;
}

// returns with cs low
void setArea (int x1, int y1, int x2, int y2) {
    cmd(0x2A);
    uint8_t xhdr [] = { (uint8_t) (x1>>8), (uint8_t) x1,
                        (uint8_t) (x2>>8), (uint8_t) x2 };
    lcd.ioRequest(IO_WRITE, xhdr, sizeof xhdr);
    cmd(0x2B);
    uint8_t yhdr [] = { (uint8_t) (y1>>8), (uint8_t) y1,
                        (uint8_t) (y2>>8), (uint8_t) y2 };
    lcd.ioRequest(IO_WRITE, yhdr, sizeof yhdr);
    cmd(0x2C);
}

void setPos (int x, int y) {
    setArea(x, y, xLimit, yLimit);
}

void pixel (int x, int y, uint16_t rgb) {
    setPos(x, y);
    out16(rgb);
    cmdEnd();
}

void fill (int x, int y, int w, int h, uint16_t rgb) {
    bounds(x+w-1, y+h-1);
    setPos(x, y);
    int n = w * h;
    while (--n >= 0)
        out16(rgb);
    cmdEnd();
}

void clear () {
    fill(0, 0, width, height, 0);
}

void orientation (uint8_t rot) {
    //width = rot & 1 ? 320 : 480;
    //height = rot & 1 ? 480 : 320;

    constexpr uint8_t mac [] = { 0x28, 0x98, 0xF8, 0x48 };
    cmd(0x36);
    lcd.ioRequest(IO_WRITE|IO_STOP, (uint8_t*) &mac[rot], 1);
}

struct Tft {
    constexpr static char mode = 'H'; // this driver uses horizontal mode
    constexpr static auto depth = 16; // colour depth (RGB565)

    static void pos (Point p) {
        setArea(p.x, p.y, width-1, height-1);
    }

    static void lim (Rect const& r) {
        setArea(r.x, r.y, r.x+r.w-1, r.y+r.h-1);
    }

    static void set (unsigned c)  { out16(c); }
    static void end ()            { cmdEnd(); }
};

TwoDee<Tft> gfx;

void testLcd () {
    spiSelect(lcd, lcdSel);

    lcdCmd.mode("P"); // MISO is reused as C/D output pin
    lcdCmd = 1;
    //lcdRst = 1;

    auto start = cycles::micros();
    init();
    logf("init %7d us", cycles::micros()-start);

    start = cycles::micros();
    clear();
    logf("clear %6d us", cycles::micros()-start);

    start = cycles::micros();
    pixel(width/2, height/2, 0xF800);
    logf("pixel %6d us", cycles::micros()-start);

    start = cycles::micros();
    auto w = gfx.writes(font, {10, 40}, "123 Hello world!");
    logf("font1 %6d us (16 ch, %d px)", cycles::micros()-start, w);
    gfx.hLine({10, 52}, w, 0xF800);

    gfx.line({10, 60}, {110, 110});
    gfx.fg = 0x07E0; // green

    gfx.fg = 0xFFE0; // yellow
    gfx.cFill({107, 80}, 20);

    gfx.fg = 0xF800; // red
    gfx.rFill({0, 0}, 50, 30, 5);

    gfx.fg = 0x001F; // blue
    gfx.bFill({28, 5}, 100, 5);

    gfx.fg = 0xFFFF; // white
    start = cycles::micros();
    auto w2 = gfx.writes(font2, {10, 114}, "123 Hello world!");
    logf("font2 %6d us (16 ch, %d px)", cycles::micros()-start, w2);
    gfx.hLine({10, 127}, w2, 0xF800);

#if !STM32F3 // FIXME ???
    start = cycles::micros();
    auto w3 = gfx.writes(font3, {64, 20}, "Hello!");
    logf("font3 %6d us (6 ch, %d px)", cycles::micros()-start, w3);
    gfx.hLine({64, 33}, w3, 0xF800);
#endif
}
