// Header file for the the central system types and functions.

[[noreturn]]
void fail (void const* addr =__builtin_return_address(0),
           char const* file =__builtin_FILE(),
           int line =__builtin_LINE());
[[noreturn]]
void hardFaultHandler (uint32_t* sp); // weak, can be redefined

void logf (char const* fmt ...);
void logDump (void const* ptr, int len =16, char const* msg =nullptr);
void logWriter (void const* ptr, size_t len); // weak, can be redefined

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

struct Message {
    uint8_t  mDst =0;
    uint8_t  mTag =0;
    uint16_t mLen =0;
    uint8_t* mPtr =nullptr;

    void*    mObj =nullptr;
    void   (*mFun)(void*,Message&) =[](void*,Message&) {}; // callback
    Message* mLnk =this;

    template< typename T >
    Message& setCallback(T* o, void (T::* f)(Message&)) {
        mObj = (uint8_t*) o;
        mFun = (void (*) (void*,Message&)) ((uint32_t*) &f)[0];
        assert(((uint32_t*) &f)[1] == 0); // TODO vtable support
        return *this;
    }

    void callback () { mFun(mObj, *this); }

    bool inUse () const { return mLnk != this; }

    Message (const Message&) =delete;
    void operator= (const Message&) =delete;
};
static_assert(sizeof (Message) == 20);

struct Chain {
    bool isEmpty () const { return cHead == nullptr; }
    Message* first () const { return cHead; }

    bool insert (Message& msg);
    bool append (Message& msg);
    bool remove (Message& msg);
    Message* pull ();

protected:
    Message* cHead =nullptr;
};
static_assert(sizeof (Chain) == 4);

namespace sys {
    enum { SLEEP, STOP0, STOP1, STOP2, STANDBY, SHUTDOWN };

    int svc (int f, int x =0, int y =0, int z =0);

    void send (Message& msg);
    Message& recv ();
    void call (Message& msg);
    bool drop (Message& msg, uint8_t id);

    void wait (uint16_t ms);
    bool coma (uint32_t sec, int mode =STOP0);

    uint8_t* pool (uint32_t bytes, uint32_t align =4);

    void init (uint32_t* ptr, uint32_t len);
    Message& fork (uint32_t*, uint16_t, int (*)(Message&), intptr_t =0);
    void quit (intptr_t ret =0);

    template< uint32_t N > // see Sys::fork comment
    void init (uint32_t (&stack)[N]) { init(stack, N); }

    // when handed an array as stack, this variant will auto-derive its size
    template< uint32_t N >
    inline static Message& fork (uint32_t (&s)[N], int (*f)(Message&), intptr_t a =0) {
        return fork(s, N, f, a);
    }

} // namespace sys

struct Fixer {
    Fixer ();
    ~Fixer ();

    bool saved;
};

struct Lock {
    bool acquire (bool blocking =true);
    void release ();

    bool locked =false;
    Chain waiting;
};

struct BlockIRQ {
    BlockIRQ () { asm ("mrs %0, primask; cpsid i" : "=r" (mask)); }
    ~BlockIRQ () { asm ("msr primask, %0" :: "r" (mask)); }
private:
    uint32_t mask;
};

struct Device {
    enum { BASE = '@', LAST = 'Z' };

    uint8_t dId;
    uint8_t dPower =sys::SLEEP; // default value: sysclk must keep running

    Device (uint8_t id);
    // TODO ~Device ();

    virtual void start (Message&) =0;
    virtual void cancel (Message&) {}
    virtual void finish () =0;

    void irqTrigger (uint8_t num);

    static Device& byId (uint8_t id);
    static uint8_t powerScan ();

protected:
    virtual bool interrupt (int) =0;

    void irqInstall (uint8_t num, uint8_t prio =0x80);
    void reply (Message* mp);
};
static_assert(sizeof (Device) == 8);

// both are called in handler mode (PendSV, via Thread::reschedule)
uint8_t lowestPower (uint8_t power, uint16_t ms); // weak, can be redefined
void resumePower ();                              // weak, can be redefined

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
    uint8_t yr, mo, dy, hh, mm, ss, ff =0;

    constexpr DateTime (int y, int m, int d, int h =0, int i =0, int s =0)
        : yr (y % 100), mo (m), dy (d), hh (h), mm (i), ss (s) {}

    // sample input: d = "Jan  1 2000", t = "12:34:56"
    constexpr DateTime (char const* d =__DATE__, char const* t =__TIME__)
        : yr (conv2d(d+9)), mo (month2d(d)), dy (conv2d(d+4)),
          hh (conv2d(t)), mm (conv2d(t+3)), ss (conv2d(t+6)) {}

    explicit DateTime (uint32_t t, uint8_t f =0) {
        ff = f;
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

    uint32_t todMillis () {
        return ((hh*60 + mm-1)*60 + ss)*1000 + (ff*1000)/256;
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

    uint32_t getReg (int reg);
    void setReg (int reg, uint32_t val);
} // namespace rtc

namespace dog {
    int resetCause (); // power: 2, nrst: 1, other: 0, watchdog: -1

    void init (int rate =6);   // max timeout, 0 ≈ 500 ms, 6 ≈ 32 s

    void reload (int n =4095); // 0..4095 x 125 µs (0) .. 8 ms (6)
    void kick ();
}

namespace cache {
#if STM32F7 || STM32H7
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
