// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "rtc"

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

//CG1 board mode
#define MODE_GPIO 1

#if MODE_GPIO
i2c::Gpio i2cBus;
#elif MODE_POLL
i2c::Poll<I2C_NAME.ADDR> i2cBus (ena::I2C_NAME, I2C_FREQ);
#elif MODE_SYNC
i2c::Sync<I2C_TYPE> i2cBus (I2C_CONF);
#elif MODE_CALL
i2c::Call<I2C_TYPE> i2cBus (I2C_CONF);
#endif

uint32_t i2cTiming (uint16_t khz, uint16_t mhz =SystemCoreClock/1'000'000) {
#if MODE_GPIO
    return khz; // i2c::Gpio estimates the delays from given khz
#else
    switch (mhz) {
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
          //CG[ i2c timing 80
          // 80 Mhz: (remove this line to re-generate)
          case  100: return 0x1000D4C0; // prs 1 tcd 0 tdd 0 scll 192 sclh 212
          case  400: return 0x00105366; // prs 0 tcd 1 tdd 0 scll 102 sclh 83
          case 1000: return 0x00101C23; // prs 0 tcd 1 tdd 0 scll 35 sclh 28
          //CG]
        }
        break;
    }
    fail();
#endif
}

void initBoard () {
    fastClock();
    rtc::init(true);
    led.mode("P");

    console.init(UART_PINS, 115'200, { UART_NAME.ADDR, ena::UART_NAME,
                                       UART_FREQ, Irq::UART_NAME, UART_CONF });
    logf("\n%s: %s @ %d MHz", PIOENV, SVDNAME, SystemCoreClock / 1'000'000);
}

extern "C" int _write (int, char* ptr, int len) {
    Message m { console.dId, 'W', (uint16_t) len, (uint8_t*) ptr };
    sys::call(m);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}
