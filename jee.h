#include <stdint.h>
#define MMIO32(x) (*(volatile uint32_t*) (x))

struct Periph {
    constexpr static uint32_t gpio = 0x40010800U;
    constexpr static uint32_t rcc  = 0x40021000U;
};

template<char port>
struct Port {
    constexpr static uint32_t base = Periph::gpio + 0x400 * (port-'A');
    constexpr static uint32_t crl  = base + 0x00;
    constexpr static uint32_t crh  = base + 0x04;
    constexpr static uint32_t idr  = base + 0x08;
    constexpr static uint32_t odr  = base + 0x0C;
    constexpr static uint32_t bsrr = base + 0x10;
    constexpr static uint32_t brr  = base + 0x14;
};

enum class Pinmode {
    in_analog        = 0b0000,
    in_float         = 0b0100,
    in_pulldown      = 0b1000,  // pseudo mode, also clears output
    in_pullup        = 0b1100,  // pseudo mode, also sets output

    out_10mhz        = 0b0001,
    out_od_10mhz     = 0b0101,
    alt_out_10mhz    = 0b1001,
    alt_out_od_10mhz = 0b1101,

    out_2mhz         = 0b0010,
    out_od_2mhz      = 0b0110,
    alt_out_2mhz     = 0b1010,
    alt_out_od_2mhz  = 0b1110,

    out              = 0b0011,
    out_od           = 0b0111,
    alt_out          = 0b1011,
    alt_out_od       = 0b1111,
};

template<char port,int pin>
struct Pin {
    typedef Port<port> gpio;
    constexpr static uint16_t mask = 1U << pin;

    static void mode (Pinmode m) {
        MMIO32(Periph::rcc + 0x18) |= 1 << (port-'A'+2); // enable GPIOx clock

        auto mval = static_cast<int>(m);
        if (mval == 0b1000 || mval == 0b1100) {
            MMIO32(gpio::bsrr) = mval & 0b0100 ? mask : mask << 16;
            mval = 0b1000;
        }

        constexpr uint32_t cr = pin & 8 ? gpio::crh : gpio::crl;
        constexpr int shift = 4 * (pin & 7);
        MMIO32(cr) = (MMIO32(cr) & ~(0xF << shift)) | (mval << shift);
    }

    static int read () {
        return mask & MMIO32(gpio::idr) ? 1 : 0;
    }

    static void write (int v) {
        // MMIO32(v ? gpio::bsrr : gpio::brr) = mask;
        // this is slightly faster when v is not known at compile time:
        MMIO32(gpio::bsrr) = v ? mask : mask << 16;
    }

    // shorthand
    operator int () const { return read(); }
    void operator= (int v) const { write(v); }

    static void toggle () {
        // both versions below are non-atomic, they access and set in two steps
        // this is smaller and faster (1.6 vs 1.2 MHz on F103 @ 72 MHz):
        // MMIO32(gpio::odr) ^= mask;
        // but this code is safer, because it can't interfere with nearby pins:
        MMIO32(gpio::bsrr) = mask & MMIO32(gpio::odr) ? mask << 16 : mask;
    }
};

template< typename MO, typename MI, typename CK, typename SS >
class SpiDev {
public:
    SpiDev () {
        nss = 1;
        nss.mode(Pinmode::out);
        sclk = 0;
        sclk.mode(Pinmode::out);
        miso.mode(Pinmode::in_float);
        mosi.mode(Pinmode::out);
    }

    static void enable () {
        nss = 0;
    }

    static void disable () {
        nss = 1;
    }

    static uint8_t transfer (uint8_t v) {
        for (int i = 0; i < 8; ++i) {
            mosi = v & 0x80;
            sclk = 1;
            v <<= 1;
            v |= miso;
            sclk = 0;
        }
        return v;
    }

    static uint8_t rwReg (uint8_t cmd, uint8_t val) {
        enable();
        transfer(cmd);
        uint8_t r = transfer(val);
        disable();
        return r;
    }

    static MO mosi;
    static MI miso;
    static CK sclk;
    static SS nss;
};

template< typename MO, typename MI, typename CK, typename SS >
MO SpiDev<MO,MI,CK,SS>::mosi;

template< typename MO, typename MI, typename CK, typename SS >
MI SpiDev<MO,MI,CK,SS>::miso;

template< typename MO, typename MI, typename CK, typename SS >
CK SpiDev<MO,MI,CK,SS>::sclk;

template< typename MO, typename MI, typename CK, typename SS >
SS SpiDev<MO,MI,CK,SS>::nss;
