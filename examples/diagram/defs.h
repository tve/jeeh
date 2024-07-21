// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "thread"

//CG1 board leds
#define LED  "B8"

constexpr Pin led (LED);

namespace jeeh {
    Pin tracePins [14];
}

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:UH7,A3"
#define UART_FREQ  170
#define UART_CONF  Irq::DMA1_CH1,Irq::DMA1_CH2,1-1,1-1,2-1,27,26
//CG]

inline Uart console ('U');

//CG[ board i2c
#define I2C_NAME  I2C1
#define I2C_PINS  "B7:OH4,A15"
#define I2C_FREQ  170
#define I2C_TYPE  I2C1.ADDR,DMA1.ADDR,3-1,4-1
#define I2C_CONF  {ena::I2C1,170,Irq::I2C1_EV,Irq::I2C1_ER}, {1-1,17,16}
//CG]

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "B5:H5,B4,B3,A11:HP"
#define SPI_FREQ  170
#define SPI_TYPE  SPI1.ADDR,DMA1.ADDR,3-1,4-1
#define SPI_CONF  {ena::SPI1,170,Irq::DMA1_CH3,Irq::DMA1_CH4}, {1-1,11,10}
//CG]

//CG: board mode

#if MODE_GPIO
i2c::Gpio i2cBus;
spi::Gpio spiBus;
#elif MODE_POLL
i2c::Poll<I2C_NAME.ADDR> i2cBus (ena::I2C_NAME, I2C_FREQ);
spi::Poll<SPI_NAME.ADDR> spiBus (ena::SPI_NAME, SPI_FREQ);
#elif MODE_SYNC
i2c::Sync<I2C_TYPE> i2cBus (I2C_CONF);
spi::Sync<SPI_TYPE> spiBus (SPI_CONF);
#elif MODE_CALL
i2c::Call<I2C_TYPE> i2cBus (I2C_CONF);
spi::Call<SPI_TYPE> spiBus (SPI_CONF);
#endif

void initBoard () {
    //fastClock();
    led.mode("P");
    Pin::config("A12:P,B0,B6,A7,A15,B7,A4,A1,A0,A8,A11,B3,B5,B4",
                    tracePins, sizeof tracePins);
    trace(INIT);

    console.init(UART_PINS, 1'000'000, { UART_NAME.ADDR, ena::UART_NAME,
                                       UART_FREQ, Irq::UART_NAME, UART_CONF });
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);

    cycles::init();
    rtc::init(false);
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { console.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

