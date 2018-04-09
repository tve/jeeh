// Driver for an ILI9341-based 320x240 LCD TFT display, connected over SPI

template< typename SPI, typename DC>
struct ILI9341 {
    constexpr static int width = 240;
    constexpr static int height = 320;

    static void init () {
        DC::write(1);
        DC::mode(Pinmode::out);
        SPI::enable();

#if 0
        { // read display ID
            cmd(0x04);
            SPI::transfer(0);
            uint32_t st = SPI::transfer(0);
            st = (st<<8) | SPI::transfer(0);
            st = (st<<8) | SPI::transfer(0);
            st = (st<<8) | SPI::transfer(0);
            printf("Disp ID: %08x\r\n", st);
            SPI::disable();
            SPI::enable();
        }
        { // read manufacturer ID
            cmd(0xDA);
            SPI::transfer(0);
            printf("Disp Manufacturer ID: %x\r\n", SPI::transfer(0));
            SPI::disable();
            SPI::enable();
        }
#endif

        static uint8_t const config [] = {
            // cmd, count, data bytes ...
            0xEF, 3, 0x03, 0x80, 0x02,               // ??
            0xCF, 3, 0x00, 0xC1, 0x30,               // power control B
            0xED, 4, 0x64, 0x03, 0x12, 0x81,         // power on sequence ctrl
            0xE8, 3, 0x85, 0x00, 0x78,               // driver timing control A
            0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,   // power control A
            0xF7, 1, 0x20,                           // pump ration control
            0xEA, 2, 0x00, 0x00,                     // driver timing control B
            0xC0, 1, 0x23,                           // PWCTR1
            0xC1, 1, 0x10,                           // PWCTR2
            0xC5, 2, 0x3e, 0x28,                     // VMCTR1
            0xC7, 1, 0x86,                           // VMCTR2
            0x36, 1, 0x68,      // memory access ctrl: column order, BGR filter
            0x37, 2, 0, 0,                           // vertical scroll start
            0x3A, 1, 0x55,                           // format: 16 bits/pixel
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

        for (uint8_t const* p = config; p < config + sizeof config; ++p) {
            cmd(*p);
            int n = *++p;
            while (--n >= 0)
                SPI::transfer(*++p);
        }
        wait_ms(120);

        bounds();
        cmd(0x29);      // DISPON
#if 0
        { // read display status
            cmd(0x09);
            SPI::transfer(0);
            uint32_t st = SPI::transfer(0);
            st = (st<<8) | SPI::transfer(0);
            st = (st<<8) | SPI::transfer(0);
            st = (st<<8) | SPI::transfer(0);
            st = (st<<8) | SPI::transfer(0);
            printf("Disp status: %08x\r\n", st);
        }
#endif

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
        out16(y);
        out16(yEnd);

        cmd(0x2B);
        out16(x);
        out16(xEnd);

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

    static void readPixels(int x, int y, int len, uint16_t *rgb) {
        SPI::enable();
        cmd(0x2A); out16(y); out16(yEnd);
        cmd(0x2B); out16(x); out16(xEnd);
        cmd(0x2E); SPI::transfer(0);
        for (int i = 0; i < len; ++i) {
            uint16_t b = SPI::transfer(0);
            rgb[i] = (b<<8) | (uint16_t)SPI::transfer(0);
        }
        SPI::disable();
    }

    static void bounds (int xend =width-1, int yend =height-1, int vscroll =0) {
        xEnd = xend;
        yEnd = yend;
        cmd(0x37);
        out16(vscroll);
    }

    static void clear () {
        SPI::enable();
        bounds();
        pixel(0, 0, 0);

        SPI::enable();
        for (int i = 1; i < width * height; ++i)
            out16(0);
        SPI::disable();
    }

    static uint16_t xEnd, yEnd;
};

template< typename SPI, typename DC>
uint16_t ILI9341<SPI,DC>::xEnd;

template< typename SPI, typename DC>
uint16_t ILI9341<SPI,DC>::yEnd;
