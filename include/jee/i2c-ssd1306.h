// Driver for an SSD1306-based 128x32 or 128x64 OLED display, using I2C.

template< typename I2C, bool BIG =false, int ADDR =0x3C >
struct SSD1306 {
    I2C& i2c;

    constexpr static int width = 128;
    constexpr static int height = BIG ? 64 : 32;

    SSD1306 (I2C& i) : i2c (i) {}

    void init () {
        static uint8_t config [] = {
            0xAE,  // DISPLAYOFF
            0xA8,  // SETMULTIPLEX
            height-1,
            0xD3,  // SETDISPLAYOFFSET
               0,
            0x40,  // SETSTARTLINE
            0x20,  // MEMORYMODE
            0x00,
            0x21,  // SET COL ADDR
               0,  // COL START
             127,  // COL END
            0xA1,  // SEGREMAP | 0x1
            0xC8,  // COMSCANDEC
            0xDA,  // SETCOMPINS
            BIG ? 0x12 : 0x02,
            0x81,  // SETCONTRAST
            BIG ? 0xCF : 0x8F,
            0xD9,  // SETPRECHARGE
            0xF1,
            0xDB,  // SETVCOMDETECT
            0x40,
            0x2E,  // STOP SCROLL
            0xD5,  // SETDISPLAYCLOCKDIV
            0x80,
            0x8D,  // CHARGEPUMP
            0x14,  // switched capacitor
            0xA4,  // DISPLAYALLON_RESUME
            0xA6,  // NORMALDISPLAY
            0xAF,  // DISPLAYON
        };

        for (int i = 0; i < (int) sizeof config; ++i)
            cmd(config[i]);
        clear();
    }

    void clear () {
        cmd(0xB0);  // SET PAGE START
        cmd(0x00);  // SETLOWCOLUMN
        cmd(0x10);  // SETHIGHCOLUMN

        i2c.start(ADDR<<1);
        i2c.write(0x40);
        for (int i = 0; i < width*height/8; ++i)
            i2c.write(0);
        i2c.stop();
    }

    // data is written in "bands" of 8 pixels high, bit 0 is the topmost line
    void copyBand (int x, int y, uint8_t const* ptr, int len) {
        cmd(0xB0 + (y>>3));   // SET PAGE START
        cmd(0x00 + (x&0xF));  // SETLOWCOLUMN
        cmd(0x10 + (x>>4));   // SETHIGHCOLUMN

        i2c.writeRegs(ADDR, 0x40, ptr, len);
    }

    void cmd (uint8_t c) {
        i2c.writeReg(ADDR, 0x80, c);
    }
};
