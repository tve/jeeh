// Driver for an ILI9341-based 320x240 LCD TFT display, connected over SPI
// FIXME untested!

template< typename MO, typename MI, typename CK, typename SS, typename DC>
struct ILI9341 {
    constexpr static int width = 240;
    constexpr static int height = 320;

    static void init () {
        dc = 1;
        dc.mode(Pinmode::out);
        spi.init();
        spi.enable();

        for (uint8_t const* p = config; p < config + sizeof config; ++p) {
            cmd(*p);
            int n = *++p;
            while (--n >= 0)
                spi.transfer(*++p);
        }

        wait_ms(120);
        cmd(0x29);      // DISPON
        spi.disable();
    }

    static void cmd (int v) {
        dc = 0;
        spi.transfer(v);
        dc = 1;
    }

    static void out16 (int v) {
        spi.transfer(v >> 8);
        spi.transfer(v);
    }

    static void pixel (int x, int y, uint16_t rgb) {
        spi.enable();

        cmd(0x2B);
        out16(x);
        out16(width-1);

        cmd(0x2A);
        out16(y);
        out16(height-1);

        cmd(0x2C);
        out16(rgb);

        spi.disable();
    }

    static void pixels (int x, int y, uint16_t const* rgb, int len) {
        pixel(y, x, *rgb);

        spi.enable();
        for (int i = 1; i < len; ++i)
            out16(rgb[i]);
        spi.disable();
    }

    static void clear () {
        pixel(0, 0, 0);

        spi.enable();
        for (int i = 1; i < width * height; ++i)
            out16(0);
        spi.disable();
    }

    static SpiDev<MO,MI,CK,SS,0> spi;
    static DC dc;
    static uint8_t const config [];
};

template< typename MO, typename MI, typename CK, typename SS, typename DC>
SpiDev<MO,MI,CK,SS,0> ILI9341<MO,MI,CK,SS,DC>::spi;

template< typename MO, typename MI, typename CK, typename SS, typename DC>
DC ILI9341<MO,MI,CK,SS,DC>::dc;

template< typename MO, typename MI, typename CK, typename SS, typename DC>
uint8_t const ILI9341<MO,MI,CK,SS,DC>::config [] = {
    0xEF, 3, 0x03, 0x80, 0x02,               // ??
    0xCF, 3, 0x00, 0xC1, 0x30,               // power control B
    0xED, 4, 0x64, 0x03, 0x12, 0x81,         // power on sequence control
    0xE8, 3, 0x85, 0x00, 0x78,               // driver timing control A
    0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,   // power control A
    0xF7, 1, 0x20,                           // pump ration control
    0xEA, 2, 0x00, 0x00,                     // driver timing control B
    0xC0, 1, 0x23,                           // PWCTR1
    0xC1, 1, 0x10,                           // PWCTR2
    0xC5, 2, 0x3e, 0x28,                     // VMCTR1
    0xC7, 1, 0x86,                           // VMCTR2
    0x36, 1, 0x68,                           // memory access ctrl: column order, BGR filter
    0x37, 2, 0, 0,                           // vertical scrolling start
    0x3A, 1, 0x55,                           // pixel format: 16 bits/pixel
    0xB1, 2, 0x00, 0x18,                     // FRMCTR1
    0xB6, 3, 0x08, 0x82, 0x27,               // DFUNCTR
    0xF2, 1, 0x00,
    0x26, 1, 0x01,                           // GAMMASET
    0xE0, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07,
              0x10, 0x03, 0x0E, 0x09, 0x00,  // GMCTRP1
    0xE1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08,
              0x0F, 0x0C, 0x31, 0x36, 0x0F,  // GMCTRN1
    0x11, 0,                                 // SLPOUT
};
