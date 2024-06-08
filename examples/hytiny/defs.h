//CG1 board leds
#define LED  "A1"

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  72
#define UART_CONF  Irq::DMA1_Channel4,Irq::DMA1_Channel5,1-1,4-1,5-1,0,0
//CG]

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "A7:5,A6:F,A5:5,A4:P"
#define SPI_FREQ  72
#define SPI_CONF  Irq::DMA1_Channel3,Irq::DMA1_Channel2,1-1,3-1,2-1,0,0
//CG]

constexpr Pin led (LED);

uint8_t initBoard (char const* app) {
    fastClock();
    led.mode("P");
    rtc::init(false);

    static Uart uart ('U');
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
    printf("\n%s: %s @ %d MHz\n", SVDNAME, app, SystemCoreClock / 1'000'000);
    return uart.dId;
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}
