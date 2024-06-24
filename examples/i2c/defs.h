// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "detect"

//CG1 board leds
#define LED  "B3"

constexpr Pin led (LED);

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A15:3"
#define UART_FREQ  80
#define UART_CONF  Irq::DMA1_CH7,Irq::DMA1_CH6,1-1,7-1,6-1,2,2
//CG]

inline Uart uart ('U');

//CG[ board i2c
#define I2C_NAME  I2C1
#define I2C_PINS  "B7:OH4,B6"
#define I2C_FREQ  80
#define I2C_TYPE  I2C1.ADDR,DMA2.ADDR,7-1,6-1
#define I2C_CONF  { ena::I2C1,80,Irq::I2C1_EV,Irq::I2C1_ER,2-1,5,5 }
//CG]

//CG: board mode

#if MODE_GPIO
I2cGpio i2c;
#elif MODE_POLL
I2cPoll<I2C_NAME.ADDR> i2c (ena::I2C_NAME, I2C_FREQ);
#elif MODE_SYNC
I2cSync<I2C_TYPE> i2c (I2C_CONF);
#elif MODE_CALL
I2cCall<I2C_TYPE> i2c (I2C_CONF);
#endif

void initBoard () {
    fastClock();
    led.mode("P");
    cycles::init();
    rtc::init(false);

    uart.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                    UART_FREQ, Irq::UART_NAME, UART_CONF });
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { uart.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}
