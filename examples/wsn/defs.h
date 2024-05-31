//CG1 board leds
#define LED  "B3"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A9:4,A10"
#define UART_FREQ  32
#define UART_CONF  Irq::DMA1_Channel4,Irq::DMA1_Channel5,1-1,4-1,5-1,4,4
//CG]

constexpr Pin led (LED);  // defined in platformio.ini

Uart uart ('U');

void logWriter (void const* ptr, size_t len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
}

void initBoard () {
    rtc::init();

    led.mode("P");  // push-pull output

    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
}
