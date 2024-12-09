// Header file for the the central system types and functions.

[[noreturn]]
void fail (void const* =__builtin_return_address(0),
            char const* =__builtin_FILE(), int =__builtin_LINE());
[[noreturn]]
void hardFaultHandler (uint32_t*); // weak, can be redefined
[[noreturn]]
void systemReset ();

uint32_t fastClock (bool high =true);
uint32_t slowClock (bool high =true);
uint32_t clockChange (uint32_t hz);

void swoInit (uint32_t baud, uint32_t hz =SystemCoreClock);
void swoWrite (void const* ptr =nullptr, size_t len =0);

void logf (char const* fmt ...);
void logDump (void const* ptr, int len =16, char const* msg =nullptr);
void logWriter (void const* ptr, size_t len); // weak, can be redefined

#define NOFLAGS 0
#if NOFLAGS
constexpr uint32_t flagsAtoZ [26] = {}; // A..Z: settings for global use
constexpr uint32_t flag (char const*) { return 0; }
#else
extern uint32_t flagsAtoZ [26]; // A..Z: settings for global use
uint32_t flag (char const* match);
#endif

template <typename T>
T take (T& x) { T r = x; x = {}; return r; }

template <typename T>
void swap (T& a, T& b) { T t = a; a = b; b = t; }

template <typename T>
void duffs (T* dst, T const* src, uint32_t count) {
    // see https://en.wikipedia.org/wiki/Duff%27s_device
    auto n = (count + 7) / 8;
    switch (count % 8) {
        case 0: do { *dst++ = *src++; [[fallthrough]];
        case 7:      *dst++ = *src++; [[fallthrough]];
        case 6:      *dst++ = *src++; [[fallthrough]];
        case 5:      *dst++ = *src++; [[fallthrough]];
        case 4:      *dst++ = *src++; [[fallthrough]];
        case 3:      *dst++ = *src++; [[fallthrough]];
        case 2:      *dst++ = *src++; [[fallthrough]];
        case 1:      *dst++ = *src++;
                } while (--n > 0);
    }
}

enum IO : uint16_t {
    IO_WRITE=1<<0, IO_READ=1<<1,
    IO_START=1<<2, IO_STOP=1<<3,
    IO_MORE=1<<4, IO_LAST=1<<5
};

struct IoReq {
    uint16_t mode;
    uint16_t len;
    uint8_t* ptr;

    IoReq (uint32_t m, uint32_t n, uint8_t* p) : mode (m), len (n), ptr (p) {}
};

template< typename T >
struct Dev : T {
    using IoSize = typename T::IoSize;

    template< uint32_t N >
    int ioRequest (IoReq const (&v) [N]) const {
        return ioRequest(v, N);
    }

    int ioRequest (IoReq const* v, IoSize n) const {
        return T::ioRequest(v, n);
    }

    int ioRequest (uint32_t m, uint8_t* p =nullptr, IoSize n =0) const {
        return T::ioRequest(m, p, n);
    }

    // simple reads and writes
    int read (void* p, IoSize n) const {
        return ioRequest(IO_START|IO_READ|IO_STOP, (uint8_t*) p, n);
    }
    int write (void const* p, IoSize n) const {
        return ioRequest(IO_START|IO_WRITE|IO_STOP, (uint8_t*) p, n);
    }

    // one byte address, single-byte data
    int readReg (uint8_t r) const {
        uint8_t v = 0;
        return readRegs(r, &v, 1) >= 0 ? v : -1;
    }
    int writeReg (uint8_t r, uint8_t v) const {
        return writeRegs(r, &v, 1);
    }

    // one byte address, read/write byte buffer
    int readRegs (uint8_t r, void* p, IoSize n) const {
        IoReq req [] = {
            { IO_START|IO_WRITE, 1, &r },
            { IO_READ|IO_STOP, n, (uint8_t*) p },
        };
        return ioRequest(req);
    }
    int writeRegs (uint8_t r, void const* p, IoSize n) const {
        IoReq req [] = {
            { IO_START|IO_WRITE|IO_MORE, 1, &r },
            { IO_WRITE|IO_STOP, n, (uint8_t*) p },
        };
        return ioRequest(req);
    }

    // two byte address, two-byte data, both big-endian
    int readReg16 (uint16_t r) const {
        uint16_t v = 0;
        return readRegs16(r, &v, 2) >= 0 ? __builtin_bswap16(v) : -1;
    }
    int writeReg16 (uint16_t r, uint16_t v) const {
        v = __builtin_bswap16(v); // send big-endian
        return writeRegs16(r, &v, 2);
    }

    // two byte big-endian address, read/write byte buffer
    int readRegs16 (uint16_t r, void* p, IoSize n) const {
        r = __builtin_bswap16(r); // send big-endian
        IoReq req [] = {
            { IO_START|IO_WRITE, 2, (uint8_t*) &r },
            { IO_READ|IO_STOP, n, (uint8_t*) p },
        };
        return ioRequest(req);
    }
    int writeRegs16 (uint16_t r, void const* p, IoSize n) const {
        r = __builtin_bswap16(r); // send big-endian
        IoReq req [] = {
            { IO_START|IO_WRITE|IO_MORE, 2, (uint8_t*) &r },
            { IO_WRITE|IO_STOP, n, (uint8_t*) p },
        };
        return ioRequest(req);
    }
};

struct BlockIRQ {
    BlockIRQ () { asm ("mrs %0, primask; cpsid i" : "=r" (mask)); }
    ~BlockIRQ () { asm ("msr primask, %0" :: "r" (mask)); }
private:
    uint32_t mask;
};

class DateTime {
    constexpr static uint8_t daysInMonth [] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };

    constexpr static uint8_t conv2d (const char* p) {
        auto v = 0;
        if ('0' <= *p && *p <= '9')
            v = *p - '0';
        return 10 * v + *++p - '0';
    };

    constexpr static uint8_t month2d (const char* p) {
        // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
        switch (p[0]) {
            case 'J': return p[1] == 'a' ? 1 : p[2] == 'n' ? 6 : 7;
            case 'F': return 2;
            case 'A': return p[2] == 'r' ? 4 : 8;
            case 'M': return p[2] == 'r' ? 3 : 5;
            case 'S': return 9;
            case 'O': return 10;
            case 'N': return 11;
            case 'D': return 12;
        }
        return 0;
    }

public:
    uint8_t yr, mo, dy, hh, mm, ss;
    uint16_t ms =0;

    constexpr DateTime (int y, int m, int d, int h =0, int i =0, int s =0)
        : yr (y % 100), mo (m), dy (d), hh (h), mm (i), ss (s) {}

    // sample input: d = "Jan  1 2000", t = "12:34:56"
    constexpr DateTime (char const* d =__DATE__, char const* t =__TIME__)
        : yr (conv2d(d+9)), mo (month2d(d)), dy (conv2d(d+4)),
          hh (conv2d(t)), mm (conv2d(t+3)), ss (conv2d(t+6)) {}

    explicit DateTime (uint32_t t, uint8_t f =0) {
        ms = f;
        ss = t % 60;
        t /= 60;
        mm = t % 60;
        t /= 60;
        hh = t % 24;
        uint16_t days = t / 24;
        uint8_t leap;
        for (yr = 0; ; ++yr) {
            leap = yr % 4 == 0;
            if (days < 365 + leap)
                break;
            days -= 365 + leap;
        }
        for (mo = 1; ; ++mo) {
            uint8_t daysPerMonth = daysInMonth[mo-1];
            if (leap && mo == 2)
                ++daysPerMonth;
            if (days < daysPerMonth)
                break;
            days -= daysPerMonth;
        }
        dy = days + 1;
    }

    constexpr operator uint32_t () const {
        uint16_t days = dy;
        for (auto i = 1; i < mo; ++i)
            days += daysInMonth[i-1];
        if (mo > 2 && yr % 4 == 0)
            ++days;
        days += 365 * yr + (yr + 3) / 4 - 1;
        return ((days * 24L + hh) * 60 + mm) * 60 + ss;
    }

    uint32_t todMillis () const {
        return ms + 1000 * (ss + 60 * (mm + 60 * hh));
    }

    struct Text {
        char buf [24]; // yyyy-mm-dd hh:mm:ss.fff

        Text (DateTime const& dt) {
            snprintf(buf, sizeof buf, "20%02d-%02d-%02d %02d:%02d:%02d.%03d",
                    dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, dt.ms);
        }
    };

    Text asText () const {
        return *this;
    }
};

namespace rtc {
    void reset ();
    void init (bool lse =true);
    void deinit ();

    bool shortSleep (uint16_t ms, int mode =0);
    bool longSleep (uint32_t sec, int mode =0);

    DateTime getDate ();
    uint32_t getSecs ();

    void set (DateTime const& dt);
    void set (uint32_t t);
    void calibrate (int diff);

    uint32_t getReg (int reg);
    void setReg (int reg, uint32_t val);
} // namespace rtc

namespace dog {
    int resetCause (); // nrst: 2, power: 1, watchdog: 0, other: -1

    void init (int rate =6);   // max timeout, 0 ≈ 500 ms, 6 ≈ 32 s

    void reload (int n =4095); // 0..4095 x 125 µs (0) .. 8 ms (6)
    void kick ();
}

namespace cache {
#if STM32F7 | STM32H7
    constexpr auto align = 32;

    void enable (); // enables both I and D caches
    void disable ();

    // instruction cache
    void invalCode (void* ptr, uint32_t len);

    // data cache
    void clean (void const* ptr, uint32_t len); // call before DMA TX
    void inval (void const* ptr, uint32_t len); // call before DMA RX
    void flush (void const* ptr, uint32_t len); // clean + inval
#else
    constexpr auto align = 4;

    inline static void enable () {}
    inline static void disable () {}
    inline static void invalCode (void const*, uint32_t) {}
    inline static void clean (void const*, uint32_t) {}
    inline static void inval (void const*, uint32_t) {}
    inline static void flush (void const*, uint32_t) {}
#endif

    template<typename T>
    void clean (T const& obj) { clean(&obj, sizeof obj); }
    template<typename T>
    void inval (T const& obj) { inval(&obj, sizeof obj); }
    template<typename T>
    void flush (T const& obj) { flush(&obj, sizeof obj); }
} // namespace cache

namespace flash {
    uint32_t pageSize (uint32_t offset);
    uint32_t& word (uint32_t offset);
    void erase (uint32_t offset);
    void write8w (uint32_t offset, uint32_t const* data);
}
