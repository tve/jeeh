// Basic 5x7 ASCII font, e.g. for OLED with copyBand support.

extern uint8_t const font5x7 [];

template< typename T, int W =6 >
struct Font5x7 {
    static void putc (int c) {
        if (c == '\n')
            x = 128;
        if (x + W > 128) {
            x = 0;
            y += 8;
            if (y >= 64)
                y = 0;
            // fill the new line with spaces
            for (int i = 0; i < 128-5; i += W)
                T::copyBand(y, i, font5x7, 5);
        }
        if (c != '\n') {
            if (c < ' ' || c > 127)
                c = 127;
            uint8_t const* p = font5x7 + 5 * (c-' ');
            T::copyBand(y, x, p, 5);
            x += W;
        }
    }

    static uint8_t y, x;
};

template< typename T, int W >
uint8_t Font5x7<T,W>::y;

template< typename T, int W >
uint8_t Font5x7<T,W>::x;
