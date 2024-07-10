// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "add"

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
