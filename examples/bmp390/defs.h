// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "spi-call"

//CG1 board leds
#define LED  "B3"

constexpr Pin led (LED);

//CG[ board uart
#define UART_NAME  USART2
#define UART_PINS  "A2:7,A15:3"
#define UART_FREQ  80
#define UART_CONF  Irq::DMA1_Channel7,Irq::DMA1_Channel6,1-1,7-1,6-1,2,2
//CG]

inline Uart console ('U');

//CG[ board i2c
#define I2C_NAME  I2C1
#define I2C_PINS  "B7:OH4,B6"
#define I2C_FREQ  80
#define I2C_TYPE  I2C1.ADDR,DMA2.ADDR,7-1,6-1
#define I2C_CONF  { ena::I2C1,80,Irq::I2C1_EV,Irq::I2C1_ER,2-1,5,5 }
//CG]

//CG[ board spi
#define SPI_NAME  SPI1
#define SPI_PINS  "B5:H5,B4,B3,A11:HP"
#define SPI_FREQ  80
#define SPI_TYPE  SPI1.ADDR,DMA1.ADDR,3-1,2-1
#define SPI_CONF  { ena::SPI1,80,Irq::DMA1_Channel3,Irq::DMA1_Channel2,1-1,1,1 }
//CG]

//CG1 board mode
#define MODE_CALL 1

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

void initBoard () {
    fastClock();
    //led.mode("P"); // conflicts with SPI
    cycles::init();
    rtc::init(false);

    console.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                       UART_FREQ, Irq::UART_NAME, UART_CONF });
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);

    // not used yet
    //Pin intr ("A8");
    //intr.mode("U");

    Pin power ("B1");
    power.mode("P");
    power = 1;
    sys::wait(5);
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { console.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

// TODO this is duplicate code, see examples/i2c/def.h - move to JeeH!

uint32_t i2cTiming (uint16_t khz, uint16_t mhz =SystemCoreClock/1'000'000) {
#if MODE_GPIO
    return khz; // i2c::Gpio estimates the delays from given khz
#else
    switch (mhz) {
      case 16: // MHz
        switch (khz) {
          //CG[ i2c timing 16
          // 16 Mhz: (remove this line to re-generate)
          case  100: return 0x00504F49; // prs 0 tcd 5 tdd 0 scll 73 sclh 79
          case  400: return 0x00500D12; // prs 0 tcd 5 tdd 0 scll 18 sclh 13
          case 1000: return 0x00500205; // prs 0 tcd 5 tdd 0 scll 5 sclh 2
          //CG]
        }
        break;
      case 80: // MHz
        switch (khz) {
          //CG[ i2c timing 80
          // 80 Mhz: (remove this line to re-generate)
          case  100: return 0x10E0D4C0; // prs 1 tcd 14 tdd 0 scll 192 sclh 212
          case  400: return 0xA0200609; // prs 10 tcd 2 tdd 0 scll 9 sclh 6
          case 1000: return 0x10E00E11; // prs 1 tcd 14 tdd 0 scll 17 sclh 14
          //CG]
        }
        break;
      case 170: // MHz
        switch (khz) {
          //CG[ i2c timing 170
          // 170 Mhz: (remove this line to re-generate)
          case  100: return 0x3010D4C1; // prs 3 tcd 1 tdd 0 scll 193 sclh 212
          case  400: return 0x0050AACE; // prs 0 tcd 5 tdd 0 scll 206 sclh 170
          case 1000: return 0x00503C49; // prs 0 tcd 5 tdd 0 scll 73 sclh 60
          //CG]
        }
        break;
    }
    fail();
#endif
}
