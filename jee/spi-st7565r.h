// Driver for Sitronix ST7565R-based 128x64 OLED display, connected over SPI
// The commands are very similar to an ssd1306

template< typename SPI, typename DC >
struct ST7565R {
    constexpr static int width = 128;
    constexpr static int height = 64;
    constexpr static int top = 0;

    static void init () {
        static uint8_t const config [] = {
            0xAE,  // Display OFF
            0xA0,  // SEG address normal order
            0xC8,  // COM direction reverse order
            0x40,  // Set start line 0
            0xA2,  // LCD bias 1/9
            0x2F,  // Power control: booster,reg,follower all on
            0x26,  // Vol reg resistor ratio
            0x81,  // Electronic volume mode
            0x11,  //    mode...
            0xAF,  // DISPLAYON
        };

        DC::write(0);
        DC::mode(Pinmode::out);
        SPI::enable();

        for (int i = 0; i < (int) sizeof config; ++i)
            SPI::transfer(config[i]);

        DC::write(1);
        SPI::disable();
    }

    static void clear () {
        // writes wrap no matter where they started
        SPI::enable();
        for (int y=0; y<height/8; y++) {
          cmd(0xB0+y); // set page start
          cmd(0x00);
          cmd(0x10);
          for (int x=0; x<width; x++)
              SPI::transfer(0);
        }
        SPI::disable();
    }

    // data is written in "bands" of 8 pixels high, bit 0 is the topmost line
    static void copyBand (int x, int y, uint8_t const* ptr, int len) {
        SPI::enable();
        cmd(0xB0 + (y>>3));   // SET PAGE START
        cmd(0x00 + (x&0xF));  // SETLOWCOLUMN
        cmd(0x10 + (x>>4));   // SETHIGHCOLUMN

        //I2C::write(0x40);
        for (int i = 0; i < len; ++i)
            SPI::transfer(ptr[i]);
        SPI::disable();
    }

    static void show64x64 (uint32_t const* logo) {
        // the input layout is nice for editing, but tricky to remap to pixels
        // ... because the respective bit and byte orders are totally different
        for (int y = 0; y < 64; y += 8) {
            uint8_t buf [64];
            for (int x = 0; x < 64; ++x)
                for (int i = 0; i < 8; ++i) {
                    buf[x] >>= 1;
                    buf[x] |= (((logo[((y+i)<<1)|(x>>5)]>>(~x&0x1F))&1) << 7);
                }
            copyBand(32, y, buf, sizeof buf);
        }
    }

    static void cmd (uint8_t v) {
        DC::write(0);
        SPI::transfer(v);
        DC::write(1);
    }

};

