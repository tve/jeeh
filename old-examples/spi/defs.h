// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "spif-call"

//CG1 board leds
#define LED  "B8"

constexpr Pin led (LED);

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:UH7,A3"
#define UART_FREQ  170
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,27,26
//CG]

inline Uart console ('U');

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "B5:H5,B4,B3,A11:HP"
#define SPI_FREQ  170
#define SPI_TYPE  SPI1.ADDR,DMA1.ADDR,3-1,4-1
#define SPI_CONF  {ena::SPI1,170,Irq::DMA1_CH3,Irq::DMA1_CH4}, {1-1,11,10}
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

void initBoard () {
    fastClock();
    led.mode("P");
    cycles::init();
    rtc::init(false);

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
