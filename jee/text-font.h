// Basic 5x7 ASCII font, e.g. for OLED with copyBand support.

extern uint8_t const font5x7 [];

template< typename T, int S =6 >
struct Font5x7 {
    constexpr static int width = T::width;
    constexpr static int height = T::height;

    static void putc (int c) {
        if (c == '\n')
            x = width;
        if (x + S > width) {
            x = 0;
            y += 8;
            if (y >= height)
                y = 0;
            // fill the new line with spaces
            for (int i = 0; i < width-5; i += S)
                T::copyBand(i, y, font5x7, 5);
        }
        if (c != '\n') {
            if (c < ' ' || c > 127)
                c = 127;
            uint8_t const* p = font5x7 + 5 * (c-' ');
            T::copyBand(x, y, p, 5);
            x += S;
        }
    }

    static uint8_t y, x;
};

template< typename T, int S >
uint8_t Font5x7<T,S>::y;

template< typename T, int S >
uint8_t Font5x7<T,S>::x;
