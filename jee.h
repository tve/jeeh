#pragma once

#include <stdarg.h>
#include <stdint.h>

#define MMIO32(x) (*(volatile uint32_t*) (x))

struct Periph {
    constexpr static uint32_t gpio = 0x40010800U;
    constexpr static uint32_t rcc  = 0x40021000U;
};

// interrupt vector table in ram

struct VTable {
    typedef void (*VTableEntry)();

    uint32_t* initial_sp_value;
    VTableEntry
        reset, nmi, hard_fault, memory_manage_fault, bus_fault, usage_fault,
        dummy_x001c[4], sv_call, debug_monitor, dummy_x0034, pend_sv, systick;
    VTableEntry
        wwdg, pvd, tamper, rtc, flash, rcc, exti0, exti1, exti2, exti3, exti4,
        dma1_channel1, dma1_channel2, dma1_channel3, dma1_channel4,
        dma1_channel5, dma1_channel6, dma1_channel7, adc1_2, usb_hp_can_tx,
        usb_lp_can_rx0, can_rx1, can_sce, exti9_5, tim1_brk, tim1_up,
        tim1_trg_com, tim1_cc, tim2, tim3, tim4, i2c1_ev, i2c1_er, i2c2_ev,
        i2c2_er, spi1, spi2, usart1, usart2, usart3, exti15_10, rtc_alarm,
        usb_wakeup, tim8_brk, tim8_up, tim8_trg_com, tim8_cc, adc3, fsmc, sdio,
        tim5, spi3, uart4, uart5, tim6, tim7, dma2_channel1, dma2_channel2,
        dma2_channel3, dma2_channel4_5, dma2_channel5, eth, eth_wkup, can2_tx,
        can2_rx0, can2_rx1, can2_sce, otg_fs;
};

extern VTable& VTableRam ();

// systick and delays

extern uint32_t volatile ticks;

extern void enableSysTick (uint32_t divider =8000000/1000);
extern void wait_ms (uint16_t ms);

// gpio

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
    constexpr static int id = 16 * (port-'A') + pin;

    static void mode (Pinmode m) {
        // enable GPIOx and AFIO clocks
        MMIO32(Periph::rcc + 0x18) |= (1 << (port-'A'+2)) | (1<<0);

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

// u(s)art

template< typename TX, typename RX >
class UartDev {
public:
    // TODO does not recognise alternate TX pins
    constexpr static int uidx = TX::id ==  9 ? 0 :  // PA9, USART1
                                TX::id ==  2 ? 1 :  // PA2, USART2
                                TX::id == 26 ? 2 :  // PB10, USART3
                                TX::id == 42 ? 3 :  // PC10, UART4
                                TX::id == 44 ? 4 :  // PC12, UART5
                                               0;   // else USART1
    constexpr static uint32_t base = uidx == 0 ? 0x40013800 :
                                                 0x40004000 + 0x400 * uidx;
    constexpr static uint32_t sr  = base + 0x00;
    constexpr static uint32_t dr  = base + 0x04;
    constexpr static uint32_t brr = base + 0x08;
    constexpr static uint32_t cr1 = base + 0x0C;

    UartDev () {
        tx.mode(Pinmode::alt_out);
        rx.mode(Pinmode::in_pullup);

        if (uidx == 0)
            MMIO32(Periph::rcc + 0x18) |= 1 << 14; // enable USART1 clock
        else
            MMIO32(Periph::rcc + 0x1C) |= 1 << (16+uidx); // UART 2..5

        MMIO32(brr) = 70;  // 115200 baud @ 8 MHz
        MMIO32(cr1) = (1<<13) | (1<<3) | (1<<2);  // UE, TE, RE
    }

    static bool writable () {
        return (MMIO32(sr) & 0x80) != 0;  // TXE
    }

    static void putc (int c) {
        while (!writable())
            ;
        MMIO32(dr) = (uint8_t) c;
    }

    static bool readable () {
        return (MMIO32(sr) & 0x24) != 0;  // RXNE or ORE
    }

    static int getc () {
        while (!readable())
            ;
        return MMIO32(dr);
    }

    static TX tx;
    static RX rx;
};

template< typename TX, typename RX >
TX UartDev<TX,RX>::tx;

template< typename TX, typename RX >
RX UartDev<TX,RX>::rx;

// interrupt-enabled uart, sits of top of polled uart

template< typename TX, typename RX, int N =50 >
class UartBufDev {
public:
    UartBufDev () {
        auto uartHandler = []() {
            if (uart.readable()) {
                int c = uart.getc();
                if (recv.free())
                    recv.put(c);
                // else discard the input
            }
            if (uart.writable()) {
                if (xmit.avail() > 0)
                    uart.putc(xmit.get());
                else
                    MMIO32(uart.cr1) &= ~(1<<7);  // disable TXEIE
            }
        };

        switch (uart.uidx) {
            case 0: VTableRam().usart1 = uartHandler; break;
            case 1: VTableRam().usart2 = uartHandler; break;
            case 2: VTableRam().usart3 = uartHandler; break;
            case 3: VTableRam().uart4  = uartHandler; break;
            case 4: VTableRam().uart5  = uartHandler; break;
        }

        // nvic interrupt numbers are 37, 38, 39, 52, and 53, respectively
        constexpr uint32_t nvic_en1r = 0xE000E104;
        constexpr int irq = (uart.uidx < 3 ? 37 : 49) + uart.uidx;
        MMIO32(nvic_en1r) |= 1 << (irq-32);  // enable USART interrupt

        MMIO32(uart.cr1) |= (1<<5);  // enable RXNEIE
    }

    static bool writable () {
        return xmit.free();
    }

    static void putc (int c) {
        while (!writable())
            ;
        xmit.put(c);
        MMIO32(uart.cr1) |= (1<<7);  // enable TXEIE
    }

    static bool readable () {
        return recv.avail() > 0;
    }

    static int getc () {
        while (!readable())
            ;
        return recv.get();
    }

    static UartDev<TX,RX> uart;
    static RingBuffer<N> recv;
    static RingBuffer<N> xmit;
};

template< typename TX, typename RX, int N >
UartDev<TX,RX> UartBufDev<TX,RX,N>::uart;

template< typename TX, typename RX, int N >
RingBuffer<N> UartBufDev<TX,RX,N>::recv;

template< typename TX, typename RX, int N >
RingBuffer<N> UartBufDev<TX,RX,N>::xmit;

// spi, bit-banged on any gpio pins

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
