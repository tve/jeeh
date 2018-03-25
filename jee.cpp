#include <jee.h>
#include <stdarg.h>
#include <string.h>

// interrupt vector table in VTableRam

VTable& VTableRam () {
    static VTable vtable __attribute__((aligned (512)));

    constexpr uint32_t ppbi  = 0xE0000000U;
    constexpr uint32_t scs   = ppbi + 0xE000;
    constexpr uint32_t scb   = scs + 0x0D00;
    constexpr uint32_t vtor  = scb + 0x8;
    constexpr uint32_t flash = 0x08000000;

    if ((void*) MMIO32(vtor) != &vtable) {
        vtable = *(VTable*) flash;
        MMIO32(vtor) = (uint32_t) &vtable;
    }

    return vtable;
}

// formatted output

static int splitInt (uint32_t val, int base, uint8_t* buf) {
    int i = 0;
    do {
        buf[i++] = val % base;
        val /= base;
    } while (val != 0);
    return i;
}

static void putFiller (void (*emit)(int), int n, char fill) {
    while (--n >= 0)
        emit(fill);
}

void putInt (void (*emit)(int), int val, int base, int width, char fill) {
    uint8_t buf [32];
    int n;
    if (val < 0) {
        n = splitInt(-val, base, buf);
        if (fill != ' ')
            emit('-');
        putFiller(emit, width - n - 1, fill);
        if (fill == ' ')
            emit('-');
    } else {
        n = splitInt(val, base, buf);
        putFiller(emit, width - n, fill);
    }
    while (n > 0) {
        uint8_t b = buf[--n];
        emit("0123456789ABCDEF"[b]);
    }
}

void veprintf(void (*emit)(int), const char* fmt, va_list ap) {
    const char* s;

    while (*fmt) {
        char c = *fmt++;
        if (c == '%') {
            char fill = *fmt == '0' ? '0' : ' ';
            int width = 0, base = 0;
            while (base == 0) {
                c = *fmt++;
                switch (c) {
                    case 'b':
                        base =  2;
                        break;
                    case 'o':
                        base =  8;
                        break;
                    case 'd':
                        base = 10;
                        break;
                    case 'x':
                        base = 16;
                        break;
                    case 'c':
                        putFiller(emit, width - 1, fill);
                        c = va_arg(ap, int);
                        // fall through
                    case '%':
                        emit(c);
                        base = 1;
                        break;
                    case 's':
                        s = va_arg(ap, const char*);
                        width -= strlen(s);
                        while (*s)
                            emit(*s++);
                        putFiller(emit, width, fill);
                        // fall through
                    default:
                        if ('0' <= c && c <= '9')
                            width = 10 * width + c - '0';
                        else
                            base = 1; // stop scanning
                }
            }
            if (base > 1) {
                int val = va_arg(ap, int);
                putInt(emit, val, base, width, fill);
            }
        } else
            emit(c);
    }
}
