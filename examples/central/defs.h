//CG[ board leds
#define LED  "A5"
#define LED1 "A5"
#define LED2 "A7"
#define LED3 "B1"
//CG]

//CG[ board uart
#define UART_NAME  USART6
#define UART_PINS  "C6:8,C7"
#define UART_FREQ  108
#define UART_CONF  Irq::DMA2_Stream6,Irq::DMA2_Stream2,2-1,6-0,2-0,5,5
//CG]

constexpr Pin led (LED);  // defined in platformio.ini

Uart uart ('U');

extern "C" int _write (int, char* ptr, int len) {
    Message m { 'U', 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

uint8_t initBoard (char const* appName) {
    fastClock();
    rtc::init();

    led.mode("P");       // push-pull output
    Pin::config("D3:P"); // ESP8266 CH_PD, power down

    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
    printf("\n%s: %s @ %d MHz\n", SVDNAME, appName, SystemCoreClock / 1'000'000);
    return uart.dId;
}
