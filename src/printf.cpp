#include "jee.h"
using namespace jeeh;
#include <cstdarg>

int veprintf (void (*fun)(void*,int), void* arg, char const* fmt, va_list ap) {
    int pad, count = 0;
    auto emit = [&](int c) { ++count; fun(arg, c); };
    auto fill = [&](int n) { while (--n >= 0) emit(pad); };

    while (*fmt)
        if (char c = *fmt++; c != '%')
            emit(c);
        else {
            pad = *fmt == '0' ? '0' : ' ';
            int width = 0, left = 0, radix = 0;
            while (radix == 0)
                switch (c = *fmt++) {
                    case '-': left = 1; break;
                    case 'o': radix = 8; break;
                    case 'u':
                    case 'd': radix = 10; break;
                    case 'p': pad = '0'; width = 8;
                              [[fallthrough]];
                    case 'x': radix = 16; break;
                    case 'c': fill(width - 1);
                              c = va_arg(ap, int);
                              [[fallthrough]];
                    case '%': emit(c); radix = 1; break;
                    case '*': width = va_arg(ap, int);
                              if (width < 0) {
                                  width = -width;
                                  left = 1;
                              }
                              break;
                    case 's': { char const* s = va_arg(ap, char const*);
                                if (s == nullptr)
                                    s = "";
                                if (width > 0 && !left) {
                                    width -= strlen(s);
                                    fill(width);
                                    width = 0;
                                }
                                while (*s) {
                                    emit(*s++);
                                    --width;
                                }
                                fill(width);
                              }
                              [[fallthrough]];
                    default:  if ('0' <= c && c <= '9')
                                width = 10 * width + c - '0';
                              else
                                radix = 1; // stop scanning
                }
            if (radix > 1) {
                uint8_t pos = 0, buf [12];
                int val = va_arg(ap, int);
                auto sign = val < 0 && c == 'd';
                uint32_t num = sign ? -val : val;
                do {
                    buf[pos++] = "0123456789ABCDEF"[num % radix];
                    num /= radix;
                } while (num != 0);
                if (sign) {
                    if (pad == ' ')
                        buf[pos++] = '-';
                    else {
                        --width;
                        emit('-');
                    }
                }
                auto n = width - pos;
                if (!left)
                    fill(n);
                while (pos > 0)
                    emit(buf[--pos]);
                if (left)
                    fill(n);
            }
        }

    return count;
}

extern "C" {

int _write (int fd, char* ptr, int len);

int printf (char const* fmt, ...) {
    // TODO figure out a way to avoid char-by-char call overhead
    auto emit = +[](void*, int c) { _write(1, (char*) &c, 1); };

    va_list ap;
    va_start(ap, fmt);
    auto n = veprintf(emit, nullptr, fmt, ap);
    va_end(ap);
    return n;
}

int vsnprintf (char* buf, size_t len, char const* fmt, va_list ap) {
    char* range [2] = { buf, buf+len-1 };
    auto emit = +[](void* p, int c) {
        auto r = (char**) p;
        if (r[0] < r[1])
            *r[0]++ = c;
        *r[0] = 0;
    };

    return veprintf(emit, range, fmt, ap);
}

} // extern "C"
