// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "recv-call"

//CG1 board leds
#define LED  "A1"

constexpr Pin led (LED);

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  72
#define UART_CONF  Irq::DMA1_Channel4,Irq::DMA1_Channel5,1-1,4-1,5-1,0,0
//CG]

inline Uart console ('U');

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "A7:5,A6:F,A5:5,A4:P"
#define SPI_FREQ  72
#define SPI_TYPE  SPI1.ADDR,DMA1.ADDR,3-1,2-1
#define SPI_CONF  {ena::SPI1,72,Irq::DMA1_Channel3,Irq::DMA1_Channel2}, {1-1,0,0}
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

//CG2 board rfm69
#define RFM69_DIOS "B5:F,B3,B4,B0,A15"
#define RFM69_NRST "A8"

constexpr Pin nrst (RFM69_NRST);
Pin dios [5];

void initBoard () {
    fastClock();

    Pin::config(RFM69_DIOS, dios, sizeof dios);
    nrst.mode("P");
    led.mode("P");  // push-pull output

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
