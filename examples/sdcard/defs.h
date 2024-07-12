// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "sdio"

//CG1 board leds
#define LED  "I1"

constexpr Pin led (LED);

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,B7"
#define UART_FREQ  108
#define UART_CONF  Irq::DMA2_Stream7,Irq::DMA2_Stream5,2-1,7-0,5-0,4,4
//CG]

inline Uart console ('U');

//CG: board spi

//CG: board mode

#if MODE_GPIO
spi::Gpio spiBus;
#elif MODE_POLL
spi::Poll<SPI_NAME.ADDR> spiBus (ena::SPI_NAME, SPI_FREQ);
#elif MODE_SYNC
spi::Sync<SPI_TYPE> spiBus (SPI_CONF);
#elif MODE_CALL
spi::Call<SPI_TYPE> spiBus (SPI_CONF);
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
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { console.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}
