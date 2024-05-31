//CG1 board leds
#define LED  "A1"

//CG[ board uart
#define UART_NAME  USART1
#define UART_PINS  "A9:7,A10"
#define UART_FREQ  72
#define UART_CONF  Irq::DMA1_Channel4,Irq::DMA1_Channel5,1-1,4-1,5-1,0,0
//CG]

constexpr Pin led (LED);

uint8_t initBoard (char const* appName) {
    fastClock();
    led.mode("P");
    rtc::init(false);

    static Uart uart ('U');
    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
    printf("\n%s: %s @ %d MHz\n", SVDNAME, appName, SystemCoreClock / 1'000'000);
    return uart.dId;
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}
