// Lines with "CG" control the code-generated parts of this file.

//CG1 board leds
#define LED  "B3"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A15:3"
#define UART_FREQ  80
#define UART_CONF  Irq::DMA1_CH7,Irq::DMA1_CH6,1-1,7-1,6-1,2,2
//CG]

//CG3 board i2c
#define I2C_NAME  I2C1
#define I2C_PINS  "B6:7,B7:3"
#define I2C_CONF  Irq::I2C1_EV,1-1,7-1,6-1,2,2

constexpr Pin led (LED);
inline Uart uart ('U');

void initBoard (char const* app) {
    fastClock();
    led.mode("P");  // push-pull output

    rtc::init(false);
    cycles::init();

    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
    logf("\n%s: %s @ %d MHz", SVDNAME, app, SystemCoreClock / 1'000'000);
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { uart.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}
