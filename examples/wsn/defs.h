//CG1 board leds
#define LED  "B3"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:4,A15"
#define UART_FREQ  32
#define UART_CONF  Irq::DMA1_Channel4,Irq::DMA1_Channel5,1-1,4-1,5-1,4,4
//CG]

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "B5:0,B4,B3,A11:P"
#define SPI_FREQ  32
#define SPI_CONF  Irq::DMA1_Channel3,Irq::DMA1_Channel2,1-1,3-1,2-1,1,1
//CG]

//CG2 board rfm69
#define RFM69_DIOS "A0:F,A1,A3,A8,B6"
#define RFM69_NRST "B1"

constexpr Pin led (LED);  // defined in platformio.ini
inline Uart uart ('U');

constexpr Pin nrst (RFM69_NRST);
Pin dios [5];

extern "C" int _write (int, char*, int) {
    fail();
}

void jeeh::logWriter (void const* ptr, size_t len) {
    Message m { uart.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
}

void initBoard (char const* app) {
    Pin::config(RFM69_DIOS, dios, sizeof dios);
    nrst.mode("P");
    led.mode("P");  // push-pull output
                    //
    rtc::init();

    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
    logf("\n%s: %s @ %d MHz\n", SVDNAME, app, SystemCoreClock / 1'000'000);
}
