// Use 2x RFM96 as spectrum waterfall display on an SPI-attached 320x240 LCD.
// See https://github.com/jeelabs/jeeh/tree/master/examples/waterfall2

#include <jee.h>
#include <jee/spi-rf96sa.h>
#include <jee/spi-ili9341.h>

UartDev< PinA<9>, PinA<10> > console;

void printf(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); veprintf(console.putc, fmt, ap); va_end(ap);
}

RF96sa< PinA<7>, PinA<6>, PinA<5>, PinA<4> > rf;
ILI9341< PinB<5>, PinB<4>, PinB<3>, PinB<0>, PinB<6> > lcd;

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
    lcd.clear();
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
    for (int i=0; i<100; i++) {
      lcd.spi.enable();
      lcd.cmd(0x37);
      lcd.out16(i);
      lcd.spi.disable();
      wait_ms(200);
    }

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
    wait_ms(1);
    lcd_reset = 1;
    // init the LCD controller
    lcd.init();
    // turn backlighting on
    PinA<15> lcd_light;
    lcd_light = 1;
    lcd_light.mode(Pinmode::out);

    printf("PB crl: 0x%08x\r\n", MMIO32(Periph::gpio+0x400+0));
    printf("PB crh: 0x%08x\r\n", MMIO32(Periph::gpio+0x400+4));
    printf("PB odr: 0x%08x\r\n", MMIO32(Periph::gpio+0x400+12));

    lcd.init();
    //testPattern();
    lcd.clear();

    initPalette();

    PinA<11> rf_reset;
    rf_reset = 0;
    rf_reset.mode(Pinmode::out);
    wait_ms(1);
    rf_reset = 1;
    wait_ms(1);
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

    static uint16_t scan [240];
    rf.setMode(rf.MODE_STANDBY);
    rf.setMode(rf.MODE_FSRX);
    rf.setMode(rf.MODE_RECEIVE);
    wait_ms(100);

    while (true) {
        uint32_t start = ticks;

        for (int x = 0; x < 320; ++x) {
            // scroll
            lcd.spi.enable();
            lcd.cmd(0x37);
            lcd.out16(x);
            lcd.spi.disable();
            printf("Scan: ");

            constexpr uint32_t middle = (912000000<<2) / (32000000 >> 11);
            constexpr uint32_t step = 164;
            uint32_t first = middle - 120 * step;

            for (int y = 0; y < 240; ++y) {
                uint32_t freq = first + y * step;
                rf.writeReg(rf.REG_FRFMSB,   freq >> 10);
                rf.writeReg(rf.REG_FRFMSB+1, freq >> 2);
                rf.writeReg(rf.REG_FRFMSB+2, freq << 6);

                uint8_t rssi = rf.readReg(rf.REG_RSSIVALUE);
                printf(" %02x", rssi);

                // add some grid points for reference
                if ((x & 0x1F) == 0 && y % 40 == 0)
                    rssi = 0xFF; // white dot
                scan[y] = palette[rssi];
            }
            printf("\r\n"); wait_ms(100);

            lcd.pixels(x, 0, scan, 240);  // update display
        }

        printf("%d ms\n", ticks - start);
    }
}
