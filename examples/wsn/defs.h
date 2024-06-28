// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "recv-call"

//CG1 board leds
#define LED  "B3"

constexpr Pin led (LED);  // defined in platformio.ini

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:4,A15"
#define UART_FREQ  32
#define UART_CONF  Irq::DMA1_Channel4,Irq::DMA1_Channel5,1-1,4-1,5-1,4,4
//CG]

inline Uart console ('U');

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "B5:0,B4,B3,A11:P"
#define SPI_FREQ  32
#define SPI_TYPE  SPI1.ADDR,DMA1.ADDR,3-1,2-1
#define SPI_CONF  { ena::SPI1,32,Irq::DMA1_Channel3,Irq::DMA1_Channel2,1-1,1,1 }
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
#define RFM69_DIOS "A0:F,A1,A3,A8,B6"
#define RFM69_NRST "B1"

constexpr Pin nrst (RFM69_NRST);
Pin dios [5];

void initBoard () {
    //fastClock(false);
    Pin::config(RFM69_DIOS, dios, sizeof dios);
    nrst.mode("P");
    led.mode("P");  // push-pull output

    rtc::init();

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
