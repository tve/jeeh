#include <jee.h>
#include <stdarg.h>
#include <string.h>

// alternate code for the 8-bit AVR and 32-bit ESP32 series

#if ARDUINO_ARCH_AVR || ARDUINO_ARCH_ESP32

void enableSysTick (uint32_t divider) {}  // ignored

void setup () {
    Serial.begin(115200);
    myMain();
}

void loop () {}

#endif // ARDUINO_ARCH_AVR || ARDUINO_ARCH_ESP32

#if __arm__

// interrupt vector table in ram

VTable& VTableRam () {
    static VTable vtable __attribute__((aligned (512)));

    // if SCB_VTOR isn't pointing to vtable, then copy current vtable to it
    VTable* volatile& vtor = *(VTable* volatile*) 0xE000ED08;
    if (vtor != &vtable) {
        vtable = *vtor;
        vtor = &vtable;
    }

    return vtable;
}

// systick and delays

uint32_t volatile ticks;

void enableSysTick (uint32_t divider) {
    VTableRam().systick = []() { ++ticks; };
    constexpr static uint32_t tick = 0xE000E000;
    MMIO32(tick + 0x14) = divider - 1;
    MMIO32(tick + 0x18) = 0;
    MMIO32(tick + 0x10) = 7;
}

void wait_ms (uint32_t ms) {
    uint32_t start = ticks;
    while ((uint32_t) (ticks - start) < ms)
        __asm("wfi");  // reduce power consumption
}

#endif // __arm__

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
    uint8_t buf [33];
    int n;
    if (val < 0 && base == 10) {
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

void veprintf(void (*emit)(int), char const* fmt, va_list ap) {
    char const* s;

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
                        s = va_arg(ap, char const*);
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
