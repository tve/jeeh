struct Periph {
    constexpr static uint32_t gpio = 0x50000000U;
    constexpr static uint32_t rcc  = 0x40021000U;
};

// interrupt vector table in ram

struct VTable {
    typedef void (*Handler)();

    uint32_t* initial_sp_value;
    Handler
        reset, nmi, hard_fault, memory_manage_fault, bus_fault, usage_fault,
        dummy_x001c[4], sv_call, debug_monitor, dummy_x0034, pend_sv, systick;
    Handler
        wwdg, pvd, rtc, flash, rcc, exti0_1, exti2_3, exti4_15, tsc,
        dma1_channel1, dma1_channel2_3, dma1_channel4_5, adc_comp, lptim1,
        usart4_5, tim2, tim3, tim6_dac, tim7, reserved4, tim21, i2c3, tim22,
        i2c1, i2c2, spi1, spi2, usart1, usart2, lpuart1_aes_rng, lcd, usb;
};

// systick and delays

extern void enableSysTick (uint32_t divider =2097000/1000);

// gpio

template<char port>
struct Port {
    constexpr static uint32_t base    = Periph::gpio + 0x400 * (port-'A');
    constexpr static uint32_t moder   = base + 0x00;
    constexpr static uint32_t typer   = base + 0x04;
    constexpr static uint32_t ospeedr = base + 0x08;
    constexpr static uint32_t pupdr   = base + 0x0C;
    constexpr static uint32_t idr     = base + 0x10;
    constexpr static uint32_t odr     = base + 0x14;
    constexpr static uint32_t bsrr    = base + 0x18;
    constexpr static uint32_t afrl    = base + 0x20;
    constexpr static uint32_t afrh    = base + 0x24;
    constexpr static uint32_t brr     = base + 0x28;
};

enum class Pinmode {
    // mode (2), typer (1), pupdr (2)
    in_analog        = 0b11000,
    in_float         = 0b00000,
    in_pulldown      = 0b00010,
    in_pullup        = 0b00001,

    out              = 0b01000,
    out_od           = 0b01100,
    alt_out          = 0b10000,
    alt_out_od       = 0b10100,
};

template<char port,int pin>
struct Pin {
    typedef Port<port> gpio;
    constexpr static uint16_t mask = 1U << pin;
    constexpr static int id = 16 * (port-'A') + pin;

    static void mode (Pinmode m, int alt =0) {
        // enable GPIOx clock
        MMIO32(Periph::rcc + 0x2C) |= 1 << (port-'A');

        auto mval = static_cast<int>(m);
        MMIO32(gpio::moder) = (MMIO32(gpio::moder) & ~(3 << 2*pin))
                            | ((mval >> 3) << 2*pin);
        MMIO32(gpio::typer) = (MMIO32(gpio::typer) & ~(1 << pin))
                            | (((mval >> 2) & 1) << pin);
        MMIO32(gpio::pupdr) = (MMIO32(gpio::pupdr) & ~(3 << 2*pin))
                            | ((mval & 3) << 2*pin);

        MMIO32(gpio::ospeedr) = (MMIO32(gpio::ospeedr) & ~(3 << 2*pin))
                              | (0b11 << 2*pin);

        constexpr uint32_t afr = pin & 8 ? gpio::afrh : gpio::afrl;
        constexpr int shift = 4 * (pin & 7);
        MMIO32(afr) = (MMIO32(afr) & ~(0xF << shift)) | (alt << shift);
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

// u(s)art

template< typename TX, typename RX >
class UartDev {
public:
    // TODO does not recognise alternate TX pins
    constexpr static int uidx = TX::id ==  9 ? 0 :  // PA9, USART1
                                TX::id ==  2 ? 1 :  // PA2, USART2
                                               0;   // else USART1
    constexpr static uint32_t base = uidx == 0 ? 0x40013800 :
                                                 0x40004000 + 0x400 * uidx;
    constexpr static uint32_t cr1 = base + 0x00;
    constexpr static uint32_t brr = base + 0x0C;
    constexpr static uint32_t isr = base + 0x1C;
    constexpr static uint32_t rdr = base + 0x24;
    constexpr static uint32_t tdr = base + 0x28;

    UartDev () {
        tx.mode(Pinmode::alt_out, 4);
        rx.mode(Pinmode::in_pullup, 4);

        if (uidx == 0)
            MMIO32(Periph::rcc + 0x34) |= 1 << 14; // enable USART1 clock
        else
            MMIO32(Periph::rcc + 0x38) |= 1 << (16+uidx); // USART 2..5

        MMIO32(brr) = 18;  // 115200 baud @ 2.1 MHz
        MMIO32(cr1) = (1<<3) | (1<<2) | (1<<0);  // TE, RE, UE
    }

    static bool writable () {
        return (MMIO32(isr) & 0x80) != 0;  // TXE
    }

    static void putc (int c) {
        while (!writable())
            ;
        MMIO32(tdr) = (uint8_t) c;
    }

    static bool readable () {
        return (MMIO32(isr) & 0x24) != 0;  // RXNE or ORE
    }

    static int getc () {
        while (!readable())
            ;
        return MMIO32(rdr);
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
        auto handler = []() {
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
            case 0: VTableRam().usart1 = handler; break;
            case 1: VTableRam().usart2 = handler; break;
        }

        // nvic interrupt numbers are 27 and 28, respectively
        constexpr uint32_t nvic_en0r = 0xE000E100;
        constexpr int irq = 27 + uart.uidx;
        MMIO32(nvic_en0r) = 1 << irq;  // enable USART interrupt

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
