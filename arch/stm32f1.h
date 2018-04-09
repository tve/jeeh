struct Periph {
    constexpr static uint32_t gpio = 0x40010800U;
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

// systick and delays

extern void enableSysTick (uint32_t divider =8000000/1000);

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
            MMIO32(Periph::rcc + 0x1C) |= 1 << (16+uidx); // U(S)ART 2..5

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
            case 2: VTableRam().usart3 = handler; break;
            case 3: VTableRam().uart4  = handler; break;
            case 4: VTableRam().uart5  = handler; break;
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

// system clock

static void enableClkAt72mhz () {
    constexpr uint32_t rcc   = 0x40021000;
    constexpr uint32_t flash = 0x40022000;

    MMIO32(flash + 0x00) = 0x12; // flash acr, two wait states
    MMIO32(rcc + 0x00) |= (1<<16); // rcc cr, set HSEON
    while ((MMIO32(rcc + 0x00) & (1<<17)) == 0) ; // wait for HSERDY
    // 8 MHz xtal src, pll 9x, pclk1 = hclk/2, adcpre = pclk2/6
    MMIO32(rcc + 0x04) = (1<<16) | (7<<18) | (4<<8) | (2<<14) | (1<<1);
    MMIO32(rcc + 0x00) |= (1<<24); // rcc cr, set PLLON
    while ((MMIO32(rcc + 0x00) & (1<<25)) == 0) ; // wait for PLLRDY
}

static int fullSpeedClock () {
    constexpr uint32_t hz = 72000000;
    enableClkAt72mhz();                 // using external 8 MHz crystal
    enableSysTick(hz/1000);             // systick once every 1 ms
    MMIO32(0x40013808) = hz/115200;     // usart1: 115200 baud @ 72 MHz
    return hz;
}

// real-time clock

struct RTC {
    constexpr static uint32_t bdcr = Periph::rcc + 0x20;
    constexpr static uint32_t pwr  = 0x40007000;
    constexpr static uint32_t rtc  = 0x40002800;
    constexpr static uint32_t crl  = rtc + 0x04;
    constexpr static uint32_t prll = rtc + 0x0C;
    constexpr static uint32_t cnth = rtc + 0x18;
    constexpr static uint32_t cntl = rtc + 0x1C;

    static void init () {
        MMIO32(Periph::rcc + 0x18) |= (0b11 << 27);  // enable PWREN and BKPEN
        MMIO32(pwr) |= (1 << 8);          // set DBP
        MMIO32(bdcr) |= (1 << 16);        // reset backup domain
        MMIO32(bdcr) &= ~(1 << 16);       // release backup domain
        MMIO32(bdcr) |= (1 << 0);         // LESON backup domain
        wait();
        MMIO32(bdcr) |= (1 << 8);         // RTSEL = LSE
        MMIO32(bdcr) |= (1 << 15);        // RTCEN
        MMIO32(crl) &= ~(1 << 3) ;        // clear RSF
        while (MMIO32(crl) & (1 << 3)) ;  // wait for RSF
        wait();
        MMIO32(crl) |= (1 << 4);          // set CNF
        MMIO32(prll) = 32767;             // set PRLL for 32 kHz crystal
        MMIO32(crl) &= ~(1 << 4);         // clear CNF
        wait();
    }

    static void wait () {
        while ((MMIO32(bdcr) & (1<<1)) == 0) ;
    }

    operator int () {
        while (true) {
            uint16_t lo = MMIO32(cntl);
            uint16_t hi = MMIO32(cnth);
            if (lo == MMIO32(cntl))
                return lo | (hi << 16);
            // if low word changed, try again
        }
    }

    void operator= (int v) {
        wait();
        MMIO32(crl) |= (1 << 4);      // set CNF
        MMIO32(cntl) = (uint16_t) v;  // set lower 16 bits
        MMIO32(cnth) = v >> 16;       // set upper 16 bits
        MMIO32(crl) &= ~(1 << 4);     // clear CNF
    }
};

// hardware spi support (TODO only SPI1 for now)

template< typename MO, typename MI, typename CK, typename SS, int CP =0 >
struct SpiHw {
    constexpr static uint32_t spi1 = 0x40013000;
    constexpr static uint32_t cr1 = spi1 + 0x00;
    constexpr static uint32_t cr2 = spi1 + 0x04;
    constexpr static uint32_t sr  = spi1 + 0x08;
    constexpr static uint32_t dr  = spi1 + 0x0C;

    static void init () {
        disable();
        SS::mode(Pinmode::out_10mhz);
        CK::mode(Pinmode::alt_out_10mhz);
        MI::mode(Pinmode::in_float);
        MO::mode(Pinmode::alt_out_10mhz);

        MMIO32(Periph::rcc + 0x18) |= (1<<12);  // SPI1EN
        // SPE, BR=2, MSTR, CPOL (clk/8, i.e. 9 MHz)
        MMIO32(cr1) = (1<<6) | (2<<3) | (1<<2) | (CP<<1);
        (void) MMIO32(sr);  // appears to be needed to avoid hang in some cases
        MMIO32(cr2) |= (1<<2);  // SSOE
    }

    static void enable () { SS::write(0); }
    static void disable () { SS::write(1); }

    static uint8_t transfer (uint8_t v) {
        MMIO32(dr) = v;
        while ((MMIO32(sr) & 1) == 0) ;
        return MMIO32(dr);
    }
};
