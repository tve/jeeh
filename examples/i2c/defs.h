// Lines with "CG" control the code-generated parts of this file.

//CG1 board leds
#define LED  "B8"

constexpr Pin led (LED);

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:UH7,A3"
#define UART_FREQ  170
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,27,26
//CG]

inline Uart uart ('U');

//CG[ board i2c
#define I2C_NAME  I2C1
#define I2C_PINS  "B7:OH4,A15"
#define I2C_FREQ  170
#define I2C_TYPE  I2C1.ADDR,DMA1.ADDR,3-1,4-1
#define I2C_CONF  { ena::I2C1,170,Irq::I2C1_EV,Irq::I2C1_ER,1-1,17,16 }
//CG]

//CG1 board mode
#define MODE_CALL (1)

#if MODE_GPIO
I2cGpio i2c;
#elif MODE_POLL
I2cPoll<I2C_NAME.ADDR> i2c (ena::I2C_NAME, I2C_FREQ);
#elif MODE_SYNC
I2cSync<I2C_TYPE> i2c (I2C_CONF);
#elif MODE_CALL
I2cCall<I2C_TYPE> i2c (I2C_CONF);
#endif

void initBoard (char const* app) {
#if !STM32G4 // FIXME
    fastClock();
#endif
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
