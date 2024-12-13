// Lines with "CG" control the code-generated parts of this file.

//CG1 pio
#define PIOENV  "sync"

//CG1 board leds
#define LED  "B8"

const Pin led (LED,"P");

//CG1 board mode
#define MODE_SYNC 1

//CG[ board pins
#define PINS_VCC "B6"
#define PINS_BMP "A11"
#define PINS_LCD "A12"
#define PINS_SRAM "B0"
#define PINS_SD "A7"
//CG]

Pin bmpVcc (PINS_VCC,"P");   // VCC for BMP390
Pin bmpSel (PINS_BMP,"U");   // NSEL for BMP390 on SPI
Pin lcdSel (PINS_LCD,"U");   // NSEL for LCD on SPI
Pin sramSel (PINS_SRAM,"U"); // NSEL for SRAM on SPI
Pin sdSel (PINS_SD,"U");     // NSEL for SD card on SPI

//CG[ board uart
#define UART_NAME USART2
#define UART_TRIGGER(w) extern "C" { \
    void USART2_IRQHandler () { (w).irqIdle(); } \
    void DMA1_Channel1_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel2_IRQHandler () { (w).irqDma(); } \
}
constexpr uart::Config UART_CONF {
    "A2:U7,A3", USART2.ADDR, ena::USART2, 170,
    DMA1.ADDR, 1-1, 1-1,2-1, 27,26,
    Irq::DMA1_CH1, Irq::DMA1_CH2, Irq::USART2,
};
//CG]

//Dev<uart::Poll<UART_CONF>> console;
Dev<uart::Sync<UART_CONF>> console;
//Dev<uart::Async<UART_CONF>> console;
//UART_TRIGGER(console)

//CG[ board spi
#define SPI_NAME SPI1
#define SPI_TRIGGER(w) extern "C" { \
    void DMA1_Channel3_IRQHandler () { (w).irqDma(); } \
    void DMA1_Channel4_IRQHandler () { (w).irqDma(); } \
}
constexpr spi::Config SPI_CONF {
    "B5:H5,B4,B3", SPI1.ADDR, ena::SPI1, 170,
    DMA1.ADDR, 1-1, 3-1,4-1, 11,10,
    Irq::DMA1_CH3, Irq::DMA1_CH4,
};
//CG]

//CG[ board i2c
#define I2C_NAME I2C1
#define I2C_TRIGGER(w) extern "C" { \
    void I2C1_EV_IRQHandler () { (w).irqI2c(); } \
}
constexpr i2c::Config I2C_CONF {
    "B7:OH4,A15", I2C1.ADDR, ena::I2C1, 170,
    DMA1.ADDR, 1-1, 3-1,4-1, 17,16,
    Irq::DMA1_CH3, Irq::DMA1_CH4, Irq::I2C1_EV, Irq::I2C1_ER,
};
//CG]

extern "C" int _write (int fd, char* buf, int len) {
    if (fd == 1 || fd == 2)
        console.write(buf, len);
    return len;
}

void initBoard () {
    fastClock(); // 160 MHz
    cycles::init();
    rtc::init(false);

    console.init(2'000'000);

    if (rtc::getSecs() == 0)
        rtc::set(DateTime{}); // set to compile date if RTC was not running

    auto dt = rtc::getDate();
    logf("\n%s: %s @ %d MHz - 20%02d-%02d-%02d %02d:%02d:%02d.%03d",
            PIOENV, SVDNAME, SystemCoreClock / 1'000'000,
            dt.yr, dt.mo, dt.dy, dt.hh, dt.mm, dt.ss, dt.ms);
    logf("\t i2c=%s  spi=%s  uart=%s",
            I2C_CONF.pins, SPI_CONF.pins, UART_CONF.pins);
    logf("\t led=%s  vcc=%s  bmp=%s  lcd=%s  sram=%s  sd=%s",
            LED, PINS_VCC, PINS_BMP, PINS_LCD, PINS_SRAM, PINS_SD);
}
