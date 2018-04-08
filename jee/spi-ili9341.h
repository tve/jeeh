// Driver for an ILI9341-based 320x240 LCD TFT display, connected over SPI
// FIXME untested!

template< typename SPI, typename DC>
struct ILI9341 {
    constexpr static int width = 320;
    constexpr static int height = 240;

    static void init () {
        static uint8_t const config [] = {
            0xEF, 3, 0x03, 0x80, 0x02,
            0xCF, 3, 0x00, 0xC1, 0x30,
            0xED, 4, 0x64, 0x03, 0x12, 0x81,
            0xE8, 3, 0x85, 0x00, 0x78,
            0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
            0xF7, 1, 0x20,
            0xEA, 2, 0x00, 0x00,
            0xC0, 1, 0x23,                           // PWCTR1
            0xC1, 1, 0x10,                           // PWCTR2
            0xC5, 2, 0x3e, 0x28,                     // VMCTR1
            0xC7, 1, 0x86,                           // VMCTR2
            0x36, 1, 0x48,                           // MADCTL
            0x37, 2, 0, 0,                           // VSCRSADD
            0x3A, 1, 0x55,                           // PIXFMT
            0xB1, 2, 0x00, 0x18,                     // FRMCTR1
            0xB6, 3, 0x08, 0x82, 0x27,               // DFUNCTR
            0xF2, 1, 0x00,
            0x26, 1, 0x01,                           // GAMMASET
            0xE0, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
                      0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,  // GMCTRP1
            0xE1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
                      0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,  // GMCTRN1
            0x11, 0,                                 // SLPOUT
        };

        DC::write(1);
        DC::mode(Pinmode::out);
        SPI::enable();

        for (uint8_t const* p = config; p < config + sizeof config; ++p) {
            cmd(*p);
            int n = *++p;
            while (--n >= 0)
                cmd(*++p);
        }
    
        wait_ms(120);
        cmd(0x29);      // DISPON
        SPI::disable();
    }

    static void cmd (int v) {
        DC::write(0);
        SPI::transfer(v);
        DC::write(1);
    }

    static void out16 (int v) {
        SPI::transfer(v >> 8);
        SPI::transfer(v);
    }

    static void pixel (int x, int y, uint16_t rgb) {
        SPI::enable();

        cmd(0x2A);
        out16(x);
        out16(width-1);

        cmd(0x2B);
        out16(y);
        out16(height-1);

        cmd(0x2C);
        out16(rgb);

        SPI::disable();
    }

    static void pixels (int x, int y, uint16_t const* rgb, int len) {
        pixel(x, y, *rgb);

        SPI::enable();
        for (int i = 1; i < len; ++i)
            out16(rgb[i]);
        SPI::disable();
    }

    static void clear () {
        pixel(0, 0, 0);

        SPI::enable();
        for (int i = 1; i < width * height; ++i)
            out16(0);
        SPI::disable();
    }
};
