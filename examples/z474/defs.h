//CG1 board leds
#define LED  "A5"

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A3"
#define UART_FREQ  170
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,27,26
//CG]

constexpr Pin led (LED);
Uart uart ('U');

extern "C" int _write (int, char* ptr, int len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

void initBoard () {
    fastClock();
    led.mode("P");
    rtc::init();

    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });

    printf("\n%s: hello @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);
}
