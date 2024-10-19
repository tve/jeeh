// Hook to a 480x320 LCD shield using 8-bit parallel over GPIO.

#include <jee.h>
#include <jee/cycles.h>
using namespace jeeh;
#include "defs.h"

#include <jee/util/twodee.h>
using namespace jeeh::twodee;

// from Oli Kraus' nice font collection, https://github.com/olikraus/u8g2/wiki
#define U8G2_FONT_SECTION(x)
#include "font.h"
Font const defaultFont (u8g2_font_lubR08_tr);

namespace {

enum { BSRR=0x18 };

Pin data [8], rst ("C1"), cs ("B0"), rs ("A4"), wr ("A1"), rd ("A0");

[[gnu::always_inline]]
inline void out8 (uint8_t v) {
#if 0
    for (auto i = 0; i < 8; ++i) {
        data[i] = v & 1;
        v >>= 1;
    }
    wr = 0;
    wr = 1;
#else
    // optimise using direct BSRR access: clear goes from 430 to 13 ms
    // bits 0..7: A9 C7 A10 B3 B5 B4 B10 A8
    GPIOA[BSRR] = (0b0000'0111'0000'0010 << 16) |
                    (((v>>0)&1) << 9) |
                    (((v>>2)&1) << 10) |
                    (((v>>7)&1) << 8);
    GPIOB[BSRR] = (0b0000'0100'0011'1000 << 16) |
                    (((v>>3)&1) << 3) |
                    (((v>>4)&1) << 5) |
                    (((v>>5)&1) << 4) |
                    (((v>>6)&1) << 10);
    GPIOC[BSRR] = (0b0000'0000'1000'0000 << 16) |
                    (((v>>1)&1) << 7);
    GPIOA[BSRR] = (0b0000'0000'0000'0010 << 16) | (1<<1); // A1
#endif
}

[[gnu::always_inline]]
inline void out16 (uint16_t v) {
    out8(v>>8);
    out8(v);
}

// returns with cs low
void cmd (uint8_t v) {
    rs = 0;
    cs = 0;
    out8(v);
    rs = 1;
}

void cmdEnd () {
    //cs = 1;
    GPIOB[BSRR] = (0b0000'0000'0000'0001 << 16) | (1<<0); // B0
}

} // namespace

void init () {
    static uint8_t const config [] = {
        // cmd, count, data bytes ...
        0xFF, 10,
#if 0
        0x3A, 1, 0x55, // pxiel format 16b
        // TODO more setup is probably needed for proper colour gamma, etc
#else
        0xF2, 9, 0x1C, 0xA3, 0x32, 0x02, 0xB2, 0x12, 0xFF, 0x12, 0x00,
        0xF1, 2, 0x36, 0xA4, 
        0xF8, 2, 0x21, 0x04, 
        0xF9, 2, 0x00, 0x08, 
        0xC0, 2, 0x0D, 0x0D, 
        0xC1, 2, 0x43, 0x00, 
        0xC2, 1, 0x00, 
        0xC5, 2, 0x00, 0x48, 
        0xE0, 15, 0x0F, 0x24, 0x1C, 0x0A, 0x0F, 0x08, 0x43, 0x88,
                    0x32, 0x0F, 0x10, 0x06, 0x0F, 0x07, 0x00,
        0xE1, 15, 0x0F, 0x38, 0x30, 0x09, 0x0F, 0x0F, 0x4E, 0x77,
                    0x3C, 0x07, 0x10, 0x05, 0x23, 0x1B, 0x00, 
        0x36, 1, 0x28, //0x0A,
        0x3A, 1, 0x55, 
#endif
        0x11, 0,       // sleep off
        0xFF, 120,
        0x29, 0        // DISPON
    };

    for (uint8_t const* p = config; p < config + sizeof config; ++p) {
        if (*p == 0xFF)
            cycles::msBusy(*++p);
        else {
            cmd(*p);
            int n = *++p;
            while (--n >= 0)
                out8(*++p);
        }
    }
    cmdEnd();
}

uint16_t width = 480, height = 320;

uint16_t xLimit = width-1;
uint16_t yLimit = height-1;

void bounds (int xend =width-1, int yend =height-1) {
    xLimit = xend;
    yLimit = yend;
}

// returns with cs low
void setPos (int x, int y) {
    cmd(0x2A);
    out16(x);
    out16(xLimit);

    cmd(0x2B);
    out16(y);
    out16(yLimit);

    cs = 0;
    cmd(0x2C);
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
    width = rot & 1 ? 320 : 480;
    height = rot & 1 ? 480 : 320;

    constexpr uint8_t mac [] = { 0x28, 0x98, 0xF8, 0x48 };
    cmd(0x36);
    out8(mac[rot]);
    cmdEnd();
}

struct Tft {
    constexpr static char mode = 'H'; // this driver uses horizontal mode
    constexpr static auto depth = 16; // colour depth (RGB565)

    static void pos (Point p) {
        cmd(0x2A); out16(p.x); out16(width-1);
        cmd(0x2B); out16(p.y); out16(height-1);
        cmd(0x2C);
    }

    static void lim (Rect const& r) {
        cmd(0x2A); out16(r.x); out16(r.x+r.w-1);
        cmd(0x2B); out16(r.y); out16(r.y+r.h-1);
        cmd(0x2C);
    }

    static void set (unsigned c)  { out16(c); }
    static void end ()            { cmdEnd(); }
};

TwoDee<Tft> gfx;

int main () {
    initBoard();

    // set all pins to push-pull output
    rst.mode("P"); rst = 0; // A.4
    cs .mode("P"); cs  = 1; // A.3
    rs .mode("P"); rs  = 1; // A.2
    wr .mode("P"); wr  = 1; // A.1
    rd .mode("P"); rd  = 1; // A.1
    Pin::config("A9:P,C7,A10,B3,B5,B4,B10,A8", data, sizeof data);
    rst = 1;

    auto start = cycles::micros();
    init();
    logf("init %6d us", cycles::micros()-start);

    start = cycles::micros();
    clear();
    logf("clear %5d us", cycles::micros()-start);

    start = cycles::micros();
    pixel(width/2, height/2, 0xF800);
    logf("pixel %5d us", cycles::micros()-start);

    for (auto seq = 0; seq < 64; ++seq) {
        if (seq % 16 == 0)
            orientation((seq / 16) % 4);
        auto v = (seq % 16) * 16;

        start = cycles::micros();
        fill(0, v, width, 16, 0xFFE0);
        fill(0, v, 16, 16, 0xF800);
        //logf("fill %6d us", cycles::micros()-start);

        cycles::msBusy(50);

        fill(0, v, width, 16, 0);
    }

    orientation(0);
    gfx.line({10, 20}, {470, 300});

    start = cycles::micros();
    gfx.fg = 0x07E0; // green
    auto w = gfx.writes(defaultFont, {320, 100}, "Hello world!");
    logf("text %6d us (12 ch)", cycles::micros()-start);
    gfx.hLine({320, 112}, w, 0xF800);

    gfx.fg = 0xFFE0; // yellow
    gfx.cFill({100, 200}, 40);

    gfx.fg = 0xF800; // red
    gfx.rFill({200, 30}, 50, 30, 10);

    gfx.fg = 0x001F; // blue
    gfx.bFill({320, 220}, 90, 30);

    while (true) {
        led.toggle();
        cycles::msBusy(250);
    }
}
