// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "uart"

//CG1 board leds
#define LED  "A8"

constexpr Pin led (LED);

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:4,A3"
#define UART_FREQ  32
#define UART_CONF  Irq::DMA1_Channel4,Irq::DMA1_Channel5,1-1,4-1,5-1,4,4
//CG]

inline Uart console ('U');

//CG[ board i2c
#define I2C_NAME  I2C1
#define I2C_PINS  "B7:OH1,B6"
#define I2C_FREQ  32
#define I2C_TYPE  I2C1.ADDR,DMA1.ADDR,6-1,7-1
#define I2C_CONF  {ena::I2C1,32,Irq::I2C1_EV,Irq::I2C1_ER}, {1-1,6,6}
//CG]

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "B5:H0,B4,B3,A15:HP"
#define SPI_FREQ  32
#define SPI_TYPE  SPI1.ADDR,DMA1.ADDR,3-1,2-1
#define SPI_CONF  {ena::SPI1,32,Irq::DMA1_Channel3,Irq::DMA1_Channel2}, {1-1,1,1}
//CG]

//CG: board mode

#if MODE_GPIO
i2c::Gpio i2cBus;
#elif MODE_POLL
i2c::Poll<I2C_NAME.ADDR> i2cBus (ena::I2C_NAME, I2C_FREQ);
#elif MODE_SYNC
i2c::Sync<I2C_TYPE> i2cBus (I2C_CONF);
#elif MODE_CALL
i2c::Call<I2C_TYPE> i2cBus (I2C_CONF);
#endif

#if MODE_GPIO
spi::Gpio spiBus;
#elif MODE_POLL
spi::Poll<SPI_NAME.ADDR> spiBus (ena::SPI_NAME, SPI_FREQ);
#elif MODE_SYNC
spi::Sync<SPI_TYPE> spiBus (SPI_CONF);
#elif MODE_CALL
spi::Call<SPI_TYPE> spiBus (SPI_CONF);
#endif

uint32_t i2cTiming (uint16_t khz, uint16_t mhz =SystemCoreClock/1'000'000) {
#if MODE_GPIO
    return khz; // i2c::Gpio estimates the delays from given khz
#else
    switch (mhz) {
      case 2: // MHz
        switch (khz) {
          //CG[ i2c timing 2
          // 2 Mhz: (remove this line to re-generate)
          case  100: return 0x00000707; // prs 0 tcd 0 tdd 0 scll 7 sclh 7
          // no valid solution for 400 kHz @ 2 MHz
          // no valid solution for 1000 kHz @ 2 MHz
          //CG]
        }
        break;
      case 4: // MHz
        switch (khz) {
          //CG[ i2c timing 4
          // 4 Mhz: (remove this line to re-generate)
          case  100: return 0x00101110; // prs 0 tcd 1 tdd 0 scll 16 sclh 17
          case  400: return 0x00100003; // prs 0 tcd 1 tdd 0 scll 3 sclh 0
          // no valid solution for 1000 kHz @ 4 MHz
          //CG]
        }
        break;
      case 16: // MHz
        switch (khz) {
          //CG[ i2c timing 16
          // 16 Mhz: (remove this line to re-generate)
          case  100: return 0x00104F49; // prs 0 tcd 1 tdd 0 scll 73 sclh 79
          case  400: return 0x00100D12; // prs 0 tcd 1 tdd 0 scll 18 sclh 13
          case 1000: return 0x00100205; // prs 0 tcd 1 tdd 0 scll 5 sclh 2
          //CG]
        }
        break;
      case 32: // MHz
        switch (khz) {
          //CG[ i2c timing 32
          // 32 Mhz: (remove this line to re-generate)
          case  100: return 0x00B0A294; // prs 0 tcd 11 tdd 0 scll 148 sclh 162
          case  400: return 0x00B01E26; // prs 0 tcd 11 tdd 0 scll 38 sclh 30
          case 1000: return 0x00B0090C; // prs 0 tcd 11 tdd 0 scll 12 sclh 9
          //CG]
        }
        break;
    }
    fail();
#endif
}

void initBoard () {
    //fastClock();
    rtc::init(true);
    led.mode("P");
    led = 1; // turn LED off

#if 0
    console.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                       UART_FREQ, Irq::UART_NAME, UART_CONF });
#endif
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}

extern "C" int _write (int, char* ptr, int len) {
#if 0
    Message m { console.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
#else
    static uart::Poll<UART_NAME.ADDR> tty (ena::UART_NAME, UART_FREQ);
    static bool inited;
    if (!inited) {
        tty.init(UART_PINS, 115'200);
        inited = true;
    }
    tty.transfer(1, (uint8_t*) ptr, len);
#endif
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}
