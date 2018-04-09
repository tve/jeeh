// Use 2x RFM96 as spectrum waterfall display on an SPI-attached 320x240 LCD.
// See https://github.com/jeelabs/jeeh/tree/master/examples/waterfall2

#include <jee.h>
#include <jee/spi-rf96sa.h>

UartDev< PinA<9>, PinA<10> > console;

void printf(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); veprintf(console.putc, fmt, ap); va_end(ap);
}

// Display

#if 0
SpiGpio< PinB<5>, PinB<4>, PinB<3>, PinB<0>, 1 > spiA;
#include <jee/spi-ili9325.h>
ILI9325< decltype(spiA) > lcd;
#else
#include <jee/spi-ili9341.h>
SpiGpio< PinB<5>, PinB<4>, PinB<3>, PinB<0>, 0 > spiA;
ILI9341< decltype(spiA), PinB<6> > lcd;
#endif

// Radio

#if 0
SpiGpio< PinA<7>, PinA<6>, PinA<5>, PinA<4> > spiB;
RF69< decltype(spiB) > rf;
#else
SpiGpio< PinA<7>, PinA<6>, PinA<5>, PinA<4> > spiB;
RF96sa< decltype(spiB) > rf;
#endif

// the range 0..255 is mapped as black -> blue -> yellow -> red -> white
// gleaned from the GQRX project by Moe Wheatley and Alexandru Csete (BSD, 2013)
// see https://github.com/csete/gqrx/blob/master/src/qtgui/plotter.cpp

static uint16_t palette [256];

static int setRgb (uint8_t r, uint8_t g, uint8_t b) {
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);  // rgb565
}

static void initPalette () {
    uint16_t* p = palette;
    for (int i = 0; i < 20; ++i)
        *p++ = setRgb(0, 0, 0);
    for (int i = 0; i < 50; ++i)
        *p++ = setRgb(0, 0, (140*i)/50);
    for (int i = 0; i < 30; ++i)
        *p++ = setRgb((60*i)/30, (125*i)/30, (115*i)/30+140);
    for (int i = 0; i < 50; ++i)
        *p++ = setRgb((195*i)/50+60, (130*i)/50+125, 255-(255*i)/50);
    for (int i = 0; i < 100; ++i)
        *p++ = setRgb(255, 255-(255*i)/100, 0);
    for (int i = 0; i < 6; ++i)
        *p++ = setRgb(255, (255*i)/5, (255*i)/5);
};

PinA<1> led;

void testPattern() {
    wait_ms(500);

    for (int x = 0; x < 240; ++x) {
        lcd.pixel(x, 0, 0xFFFF);
        lcd.pixel(x, 319, 0xFFFF);
    }
    for (int y = 0; y < 320; ++y) {
        lcd.pixel(0, y, 0xFFFF);
        lcd.pixel(239, y, 0xFFFF);
    }
    for (int i=0; i<100; i++) {
      lcd.pixel(i, i, 0xF800);
      lcd.pixel(i, 2*i, 0xF800);
    }
    for (int i=101; i<240; i++) {
      lcd.pixel(i, i, 0x03E0);
    }
    for (int i=0; i<80; i++) {
      lcd.pixel(i+240, i, 0xFFE0);
      lcd.pixel(i, i+240, 0x001F);
    }
    wait_ms(1000);
#if 0
    for (int i=0; i<100; i++) {
      lcd.spi.enable();
      lcd.cmd(0x37);
      lcd.out16(i);
      lcd.spi.disable();
      wait_ms(200);
    }
#endif

    while (true) {
        printf("\r%d", ticks);
        led.toggle();
        wait_ms(500);
    }
}

int main () {
    fullSpeedClock();
    //enableSysTick();
    printf("\r\n===== Waterfall 2 starting...\r\n");

    // disable JTAG in AFIO-MAPR to release PB3, PB4, and PA15
    constexpr uint32_t afio = 0x40010000;
    MMIO32(afio + 0x04) |= 1 << 25; // disable JTAG, keep SWD enabled

    // rtp touch screen is on the same SPI bus, make sure it's disabled
    //PinB<2> rtpcs;
    //rtpcs = 1;
    //rtpcs.mode(Pinmode::out);

    // handle a couple of extra LCD pins that the driver doesn't deal with...
    // start with a reset pulse
    PinB<7> lcd_reset;
    lcd_reset = 0;
    lcd_reset.mode(Pinmode::out);
    spiA.init();
    wait_ms(2);
    lcd_reset = 1;
    wait_ms(10); // data sheet says to wait 5ms
    // init the LCD controller
    lcd.init();
    // turn backlighting on
    PinA<15> lcd_light;
    lcd_light = 1;
    lcd_light.mode(Pinmode::out);

    printf("PB crl: 0x%08x\r\n", MMIO32(Periph::gpio+0x400+0));
    printf("PB crh: 0x%08x\r\n", MMIO32(Periph::gpio+0x400+4));
    printf("PB odr: 0x%08x\r\n", MMIO32(Periph::gpio+0x400+12));

    lcd.clear();
    //testPattern();

    initPalette();

    PinA<11> rf_reset;
    rf_reset = 0;
    rf_reset.mode(Pinmode::out);
    wait_ms(1);
    rf_reset = 1;
    wait_ms(1);
    spiB.init();
    rf.init(1, true);    // init for 10Khz steps
    rf.setFrequency(912); // start at 912Mhz for now...

    printf("Radio rev: %02x\r\n", rf.readReg(0x42));

    // dump all radio registers
    printf("   ");
    for (int i = 0; i < 16; ++i)
        printf("%3x", i);
    for (int i = 0; i < 0x80; i += 16) {
        printf("\r\n%02x:", i);
        for (int j = 0; j < 16; ++j)
            printf(" %02x", rf.readReg(i+j));
    }
    printf("\r\n");
    wait_ms(500);

    static uint16_t pixelRow [lcd.width];
    rf.setMode(rf.MODE_RECEIVE);
    wait_ms(10);

    while (true) {
        uint32_t start = ticks;

        for (int y = 0; y < lcd.height; ++y) {

            constexpr uint32_t middle = (912000000<<2) / (32000000 >> 11);
            constexpr uint32_t step = 164;
            uint32_t first = middle - 120 * step;

            //printf("Scan: ");
            for (int x = 0; x < lcd.width; ++x) {
                uint32_t freq = first + x * step;
                rf.writeReg(rf.REG_FRFMSB,   freq >> 10);
                rf.writeReg(rf.REG_FRFMSB+1, freq >> 2);
                rf.writeReg(rf.REG_FRFMSB+2, freq << 6);

                uint8_t rssi = rf.readReg(rf.REG_RSSIVALUE);
                //printf(" %02x", rssi);

                // add some grid points for reference
                if ((y & 0x1F) == 0 && x % 40 == 0)
                    rssi = 0xFF; // white dot
                pixelRow[x] = palette[rssi];
            }
            //printf("\r\n"); wait_ms(100);

            lcd.bounds(lcd.width-1, y, y);  // write one line and set scroll
            lcd.pixels(0, y, pixelRow, lcd.width);  // update display
        }

        printf("screen:%dms sweep:%dms\r\n", ticks - start, (ticks-start)/lcd.height);
    }
}
