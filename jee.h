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

// slowed-down pin, adds a configurable delay after setting the pin

template< typename T, int N >
struct SlowPin : public T {

    static void write (int v) {
        T::write(v);
        for (int i = 0; i < N; ++i)
            __asm("");  // avoid getting optimised away
    }

    // shorthand
    void operator= (int v) const { write(v); }
};

// spi, bit-banged on any gpio pins

template< typename MO, typename MI, typename CK, typename SS, int CP =0 >
class SpiDev {
public:
    SpiDev () {
        nsel = 1;
        nsel.mode(Pinmode::out);
        sclk = CP;
        sclk.mode(Pinmode::out);
        miso.mode(Pinmode::in_float);
        mosi.mode(Pinmode::out);
    }

    static void enable () { nsel = 0; }
    static void disable () { nsel = 1; }

    static uint8_t transfer (uint8_t v) {
        for (int i = 0; i < 8; ++i) {
            mosi = v & 0x80;
            sclk = !CP;
            v <<= 1;
            v |= miso;
            sclk = CP;
        }
        return v;
    }

    static MO mosi;
    static MI miso;
    static CK sclk;
    static SS nsel;
};

template< typename MO, typename MI, typename CK, typename SS, int CP >
MO SpiDev<MO,MI,CK,SS,CP>::mosi;

template< typename MO, typename MI, typename CK, typename SS, int CP >
MI SpiDev<MO,MI,CK,SS,CP>::miso;

template< typename MO, typename MI, typename CK, typename SS, int CP >
CK SpiDev<MO,MI,CK,SS,CP>::sclk;

template< typename MO, typename MI, typename CK, typename SS, int CP >
SS SpiDev<MO,MI,CK,SS,CP>::nsel;

// i2c, bit-banged on any gpio pins

template< typename SDA, typename SCL, int N =0 >
class I2cBus {
    static void sclHi () { scl = 1; while (!scl) ; }

public:
    I2cBus () {
        sda = 1;
        sda.mode(Pinmode::out_od);
        scl = 1;
        scl.mode(Pinmode::out_od);
    }

    static uint8_t start(int addr) {
        scl = 0;
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
        scl = 0;
        for (int mask = 0x80; mask != 0; mask >>= 1) {
            sda = data & mask;
            sclHi();
            scl = 0;
        }
        sda = 1;
        sclHi();
        bool ack = !sda;
        scl = 0;
        return ack;
    }

    static int read(bool last) {
        int data = 0;
        for (int mask = 0x80; mask != 0; mask >>= 1) {
            sclHi();
            if (sda)
                data |= mask;
            scl = 0;
        }
        sda = last;
        sclHi();
        scl = 0;
        if (last)
            stop();
        return data;
    }

    static SDA sda;
    static SlowPin<SCL,N> scl;
};

template< typename SDA, typename SCL, int N >
SDA I2cBus<SDA,SCL,N>::sda;

template< typename SDA, typename SCL, int N >
SlowPin<SCL,N> I2cBus<SDA,SCL,N>::scl;

// formatted output

extern void putInt (void (*emit)(int), int val, int base =10, int width =0, char fill =' ');
extern void veprintf(void (*emit)(int), const char* fmt, va_list ap);
