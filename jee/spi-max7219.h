// Driver for one or more 8x8 matrix displays based on the MAX7219 chip.
// see https://jeelabs.org/ref/MAX7219.pdf

template< typename SPI, int NUM =1 >
struct MAX7219 {
    static void init () {
        sendAll(0xC, 0x01);
        sendAll(0xB, 0x07);
        intensity(0);
        clear();
    }

    static void pixel (int x, int y, int on) {
        int t = x & 7;
        x = (x & ~7) | (y ^ 7);
        y = t;

        if (on)
            buf[x] |= (1<<y);
        else
            buf[x] &= ~(1<<y);

        sendOne(x>>3, (x&7)+1, buf[x]);
    }

    static void clear () {
        for (int i = 1; i <= 8; ++i)
            sendAll(i, 0);
    }

    static void intensity (int i) {
        sendAll(0xA, i);
    }

    static void sendAll (uint8_t reg, uint8_t val) {
        SPI::enable();
        for (int i = 0; i < NUM; ++i) {
            SPI::transfer(reg);
            SPI::transfer(val);
        }
        SPI::disable();
    }

    static void sendOne (int idx, uint8_t reg, uint8_t val) {
        SPI::enable();
        for (int i = 0; i < NUM; ++i) {
            SPI::transfer(i == idx ? reg : 0);
            SPI::transfer(val);
        }
        SPI::disable();
    }

    static uint8_t buf [8*NUM];
};

template< typename SPI, int NUM >
uint8_t MAX7219<SPI,NUM>::buf [8*NUM];
