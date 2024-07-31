// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "spif-call"

//CG1 board leds
#define LED  "A1"

constexpr Pin led (LED);

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  84
#define UART_CONF  Irq::DMA2_Stream7,Irq::DMA2_Stream5,2-1,7-0,5-0,4,4
//CG]

inline Uart console ('U');

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "B5:H5,B4,B3,A15:HP"
#define SPI_FREQ  84
#define SPI_TYPE  SPI1.ADDR,DMA2.ADDR,3-0,2-0
#define SPI_CONF  {ena::SPI1,84,Irq::DMA2_Stream3,Irq::DMA2_Stream2}, {2-1,3,3}
//CG]

//CG1 board mode
#define MODE_CALL 1

#if MODE_GPIO
spi::Gpio spiBus;
#elif MODE_POLL
spi::Poll<SPI_NAME.ADDR> spiBus (ena::SPI_NAME, SPI_FREQ);
#elif MODE_SYNC
spi::Sync<SPI_TYPE> spiBus (SPI_CONF);
#elif MODE_CALL
spi::Call<SPI_TYPE> spiBus (SPI_CONF);
#endif

template< typename SPI >
struct SpifStorage : SpiFlash<SPI> {
    using BASE = SpiFlash<SPI>;
    uint32_t base, size;

    SpifStorage (SPI& spi, uint32_t off , uint32_t len)
            : BASE (spi), base (off), size (len) {
        // both limits must be on a page boundary
        assert(base % pageSize(0) == 0);
        assert((base + len) % pageSize(len) == 0);
    }

    uint32_t pageSize (uint32_t) const {
        return 4096;
    }

    void wipe () const {
        for (auto i = 0U; i < size; i += pageSize(i))
            erase(i);
    }

    void erase (uint32_t pos) const {
        assert((base + pos) % pageSize(pos) == 0);
        BASE::erase(base + pos);
    }

    uint32_t const* map (uint32_t pos) const {
        return nullptr;
    }

    uint32_t read (uint32_t pos, void* ptr, uint32_t len) const {
        BASE::read(base + pos, (uint8_t*) ptr, len);
        return len;
    }

    uint32_t write (uint32_t pos, void const* ptr, uint32_t len) const {
        BASE::write(base + pos, (uint8_t const*) ptr, len);
        return len;
    }
};

struct FlashStorage {
    uint32_t base, size;

    FlashStorage (uint32_t off , uint32_t len) : base (off), size (len) {
        // both limits must be on a page boundary
        assert(base % pageSize(0) == 0);
        assert((base + len) % pageSize(len) == 0);
    }

    uint32_t pageSize (uint32_t pos) const {
        return flash::pageSize(base + pos);
    }

    void wipe () const {
        for (auto i = 0U; i < size; i += pageSize(i))
            erase(i);
    }

    void erase (uint32_t pos) const {
        assert((base + pos) % pageSize(pos) == 0);
        flash::erase(base + pos);
    }

    uint32_t const* map (uint32_t pos) const {
        return &flash::word(base + pos);
    }

    uint32_t read (uint32_t pos, void* ptr, uint32_t len) const {
        memcpy(ptr, map(pos), len);
        return len;
    }

    uint32_t write (uint32_t pos, void const* ptr, uint32_t len) const {
        assert(pos % 32 == 0 && len % 32 == 0);
        auto p = (uint32_t*) ptr; // TODO may be unaligned, not ok on M0+
        for (auto i = 0U; i < len; i += 32)
            flash::write8w(base + pos, p + i/4);
        return len;
    }
};

#if MODE_GPIO | MODE_POLL | MODE_SYNC | MODE_CALL
SpifStorage fs { spiBus, 0x0'0000, 0x1'0000 };
#else
FlashStorage fs { 0x1'0000, 0x1'0000 };
#endif

void initBoard () {
    fastClock();
    cycles::init();
    rtc::init(false);
    led.mode("P");
    led = 1; // inverted logic

    console.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                       UART_FREQ, Irq::UART_NAME, UART_CONF });
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);

#if MODE_GPIO | MODE_POLL | MODE_SYNC | MODE_CALL
    spiBus.init(SPI_PINS, 100'000);
#endif
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { console.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}
