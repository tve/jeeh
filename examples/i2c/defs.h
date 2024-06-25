// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "fram-poll"

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
#define MODE_POLL 1

#if MODE_GPIO
I2cGpio i2c;
#elif MODE_POLL
I2cPoll<I2C_NAME.ADDR> i2c (ena::I2C_NAME, I2C_FREQ);
#elif MODE_SYNC
I2cSync<I2C_TYPE> i2c (I2C_CONF);
#elif MODE_CALL
I2cCall<I2C_TYPE> i2c (I2C_CONF);
#endif

uint32_t i2cTiming (uint16_t mhzSys, uint16_t khzBus) {
    switch (mhzSys) {
      case 16: // MHz
        switch (khzBus) {
          //CG[ i2c timing 16
          // 16 Mhz: (remove this line to re-generate)
          case  100: return 0x00504F49; // prs 0 tcd 5 tdd 0 scll 73 sclh 79
          case  400: return 0x00500D12; // prs 0 tcd 5 tdd 0 scll 18 sclh 13
          case 1000: return 0x00500205; // prs 0 tcd 5 tdd 0 scll 5 sclh 2
          //CG]
        }
        break;
      case 80: // MHz
        switch (khzBus) {
          //CG[ i2c timing 80
          // 80 Mhz: (remove this line to re-generate)
          case  100: return 0x10E0D4C0; // prs 1 tcd 14 tdd 0 scll 192 sclh 212
          case  400: return 0xA0200609; // prs 10 tcd 2 tdd 0 scll 9 sclh 6
          case 1000: return 0x10E00E11; // prs 1 tcd 14 tdd 0 scll 17 sclh 14
          //CG]
        }
        break;
      case 170: // MHz
        switch (khzBus) {
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
}

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
