// Basic 5x7 ASCII font, e.g. for OLED with copyBand support.

extern uint8_t const font5x7 [];

template< typename T, int W =128, int H =64, int S =6 >
struct Font5x7 {
    static void putc (int c) {
        if (c == '\n')
            x = W;
        if (x + S > W) {
            x = 0;
            y += 8;
            if (y >= H)
                y = 0;
            // fill the new line with spaces
            for (int i = 0; i < W-5; i += S)
                T::copyBand(y, i, font5x7, 5);
        }
        if (c != '\n') {
            if (c < ' ' || c > 127)
                c = 127;
            uint8_t const* p = font5x7 + 5 * (c-' ');
            T::copyBand(y, x, p, 5);
            x += S;
        }
    }

    static uint8_t y, x;
};

template< typename T, int W, int H, int S >
uint8_t Font5x7<T,W,H,S>::y;

template< typename T, int W, int H, int S >
uint8_t Font5x7<T,W,H,S>::x;
