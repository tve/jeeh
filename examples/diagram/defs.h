// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "lptimer"

//CG1 board leds
#define LED  "B3"

constexpr Pin led (LED);

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A15:3"
#define UART_FREQ  80
#define UART_CONF  Irq::DMA1_CH7,Irq::DMA1_CH6,1-1,7-1,6-1,2,2
//CG]

inline Uart console ('U');

//CG[ board i2c
#define I2C_NAME  I2C1
#define I2C_PINS  "B7:O4,B6"
#define I2C_FREQ  80
#define I2C_TYPE  I2C1.ADDR,DMA2.ADDR,7-1,6-1
#define I2C_CONF  {ena::I2C1,80,Irq::I2C1_EV,Irq::I2C1_ER}, {2-1,5,5}
//CG]

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "B5:H5,B4,B3,A11:HP"
#define SPI_FREQ  80
#define SPI_TYPE  SPI1.ADDR,DMA1.ADDR,3-1,2-1
#define SPI_CONF  {ena::SPI1,80,Irq::DMA1_Channel3,Irq::DMA1_Channel2}, {1-1,1,1}
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

namespace jeeh {
    Pin tracePins [14];
}

void initBoard () {
    led.mode("P");

#if STM32L432xx
    //RCC[0x00](8,4) = 0b1001;      // HSIASFS HSIKERON HSION
    RCC[0x00](8) = 1;             // HSION
    while (RCC[0x00](10) == 0) {} // wait for HSIRDY
    //RCC[0x88](2,2) = 2;           // use HSI16 for USART2
    RCC[0x08](0,2) = 1;           // use HSI16 as system clock
    RCC[0x08](15) = 1;            // use HSI16 on wakeup from stop
    SystemCoreClock = 16'000'000;

    // LED shared with SPI1 SCK
    Pin::config("A12:P,B0,B1,A7,A6,A5,A4,A1,A0,A8,A11,B3,B5,B4",
                    tracePins, sizeof tracePins);
    rtc::init(true); // has 32 kHz XTAL
#elif STM32G431xx
    Pin::config("A12:P,B0,B6,A7,A15,B7,A4,A1,A0,A8,A11,B3,B5,B4",
                    tracePins, sizeof tracePins);
    rtc::init(false);
#endif

    trace(INIT);

    console.init(UART_PINS, 1'000'000, { UART_NAME.ADDR, ena::UART_NAME,
                                       UART_FREQ, Irq::UART_NAME, UART_CONF });
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);

    cycles::init();
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { console.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

