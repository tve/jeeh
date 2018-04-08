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

    static uint16_t y, x;
};

template< typename T, int S >
uint16_t Font5x7<T,S>::y;

template< typename T, int S >
uint16_t Font5x7<T,S>::x;

// optional adapter to print text on pixel-oriented displays

template< typename LCD >
struct TextLcd : LCD {
    // TODO write the vertically and enable hardware vertical scrolling
    static void copyBand (int x, int y, uint8_t const* ptr, int len) {
        // TODO this is very inefficient code, but it avoids an extra buffer
        for (int xi = 0; xi < len; ++xi) {
            uint8_t v = *ptr++;
            for (int yi = 0; yi < 8; ++yi) {
                // FIXME vertical axis has to be flipped, why?
                LCD::pixel(x+xi, LCD::height-1-(y+yi), v&1 ? 0xFFFF : 0x0000);
                v >>= 1;
            }
        }
    }
};
