#pragma once

#include <stdarg.h>
#include <stdint.h>

#define MMIO32(x) (*(volatile uint32_t*) (x))

// general-purpose ring buffer

template< int N >
class RingBuffer {
    uint16_t volatile in, out;
    uint8_t volatile buf [N];

public:
    RingBuffer () : in (0), out (0) {}

    int avail () const {
        int r = in - out;
        return r >= 0 ? r : r + N;
    }

    bool free () const {
        return avail() < N-1;
    }

    void put (uint8_t v) {
        uint16_t pos = in;
        buf[pos++] = v;
        in = pos < N ? pos : 0;
    }

    uint8_t get () {
        uint16_t pos = out;
        uint8_t v = buf[pos++];
        out = pos < N ? pos : 0;
        return v;
    }
};

// interrupt vector table in ram

struct VTable;
extern VTable& VTableRam ();

// architecture-specific definitions

#if STM32F1
#include "arch/stm32f1.h"
#endif
#if STM32F4
#include "arch/stm32f4.h"
#endif
#if STM32L0
#include "arch/stm32l0.h"
#endif
#if STM32L4
#include "arch/stm32l4.h"
#endif
#if ARDUINO_ARCH_AVR
#include "arch/avr328.h"
#endif
#if ARDUINO_ARCH_ESP32
#include "arch/esp32.h"
#endif

// shorthand: PinA<1> expands to Pin<'A',1>, etc

template <int N> using PinA = Pin<'A',N>;
template <int N> using PinB = Pin<'B',N>;
template <int N> using PinC = Pin<'C',N>;
template <int N> using PinD = Pin<'D',N>;
template <int N> using PinE = Pin<'E',N>;
template <int N> using PinF = Pin<'F',N>;
template <int N> using PinG = Pin<'G',N>;
template <int N> using PinH = Pin<'H',N>;
template <int N> using PinI = Pin<'I',N>;
template <int N> using PinJ = Pin<'J',N>;
template <int N> using PinK = Pin<'K',N>;

// systick and delays

#ifndef ticks
extern uint32_t volatile ticks;
#endif

extern void wait_ms (uint32_t ms);

// spi, bit-banged on any gpio pins

template< typename MO, typename MI, typename CK, typename SS >
class SpiDev {
public:
    SpiDev () {
        nsel = 1;
        nsel.mode(Pinmode::out);
        sclk = 0;
        sclk.mode(Pinmode::out);
        miso.mode(Pinmode::in_float);
        mosi.mode(Pinmode::out);
    }

    static void enable () { nsel = 0; }
    static void disable () { nsel = 1; }

    static uint8_t transfer (uint8_t v) {
        for (int i = 0; i < 8; ++i) {
            mosi = v & 0x80;
            sclk.toggle();
            v <<= 1;
            v |= miso;
            sclk.toggle();
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
    static SS nsel;
};

template< typename MO, typename MI, typename CK, typename SS >
MO SpiDev<MO,MI,CK,SS>::mosi;

template< typename MO, typename MI, typename CK, typename SS >
MI SpiDev<MO,MI,CK,SS>::miso;

template< typename MO, typename MI, typename CK, typename SS >
CK SpiDev<MO,MI,CK,SS>::sclk;

template< typename MO, typename MI, typename CK, typename SS >
SS SpiDev<MO,MI,CK,SS>::nsel;

// i2c, bit-banged on any gpio pins

template< typename SDA, typename SCL >
class I2cDev {
    static void hold () {
        // TODO make configurabe, this is ≈ 360 kHz for STM32F1 @ 72 MHz
        for (int i = 0; i < 5; ++i)
            __asm("");
    }

    static void sclHi () { scl = 1; while (!scl); hold(); }
    static void sclLo () { scl = 0; hold(); }

public:
    I2cDev () {
        sda = 1;
        sda.mode(Pinmode::out_od);
        scl = 1;
        scl.mode(Pinmode::out_od);
    }

    static uint8_t start(int addr) {
        sclLo();
        sclHi();
        sda = 0;
        return write(addr);
    }

    static void stop() {
        sda = 0;
        sclHi();
        sda = 1;
    }

    static bool write(int data) {
        sclLo();
        for (int mask = 0x80; mask != 0; mask >>= 1) {
            sda = data & mask;
            sclHi();
            sclLo();
        }
        sda = 1;
        sclHi();
        bool ack = !sda;
        sclLo();
        return ack;
    }

    static int read(bool last) {
        int data = 0;
        for (int mask = 0x80; mask != 0; mask >>= 1) {
            sclHi();
            if (sda)
                data |= mask;
            sclLo();
        }
        sda = last;
        sclHi();
        sclLo();
        if (last)
            stop();
        sda = 1;
        return data;
    }

    static SDA sda;
    static SCL scl;
};

template< typename SDA, typename SCL >
SDA I2cDev<SDA,SCL>::sda;

template< typename SDA, typename SCL >
SCL I2cDev<SDA,SCL>::scl;

// formatted output

extern void putInt (void (*emit)(int), int val, int base =10, int width =0, char fill =' ');
extern void veprintf(void (*emit)(int), const char* fmt, va_list ap);
