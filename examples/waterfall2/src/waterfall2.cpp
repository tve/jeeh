// Use 2x RFM96 as spectrum waterfall display on an SPI-attached 320x240 LCD.
// See https://github.com/jeelabs/jeeh/tree/master/examples/waterfall2

#include <jee.h>
#include <jee/spi-rf96sa.h>
#include <jee/text-font.h>

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

TextLcd< decltype(lcd) > text;
Font5x7< decltype(text) > lcd_text;
void lcd_printf(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt); veprintf(lcd_text.putc, fmt, ap); va_end(ap);
}

// Radio

#if 0
SpiGpio< PinA<7>, PinA<6>, PinA<5>, PinA<4> > spiB;
RF69< decltype(spiB) > rf;
#else
SpiHw< PinA<7>, PinA<6>, PinA<5>, PinA<4> > spiB;
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

void initLcd() {
    // rtp touch screen is on the same SPI bus, make sure it's disabled
    //PinB<2> rtpcs;
    //rtpcs = 1;
    //rtpcs.mode(Pinmode::out);

    // handle a couple of extra LCD pins that the driver doesn't deal with...
    // start with a reset pulse
    PinB<7> lcd_reset;
    lcd_reset = 0;
    lcd_reset.mode(Pinmode::out);
    // init SPI and end reset
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
    lcd.clear();
}

void sweepDisplay(int y, int count, uint8_t buf[]) {
    uint16_t pixelRow[count];
    for (int x = 0; x < count; ++x)
        pixelRow[x] = palette[(uint8_t)(~buf[x])];
    if ((y & 0x1F) == 0) {
        for (int x=0; x<count; x+=count/4)
            pixelRow[x] = 0xFFFF; // white dot
    }
    lcd.bounds(count-1, y, y);     // write one line and set scroll
    lcd.pixels(0, y, pixelRow, count);  // update display
}

void initRadio(uint8_t config) {
    // issue reset pulse
    PinA<11> rf_reset;
    rf_reset = 0;
    rf_reset.mode(Pinmode::out);
    wait_ms(1);
    // init SPI and end reset
    spiB.init();
    rf_reset = 1;
    wait_ms(1);
    // init radio itself
    rf.init(config, true);    // init for 10Khz steps
    rf.setFrequency(915); // start at 915Mhz for now...
    printf("Radio rev: %02x\r\n", rf.readReg(0x42));
}

// dump all radio registers
void dumpRadioRegs() {
    printf("   ");
    for (int i = 0; i < 16; ++i)
        printf("%3x", i);
    for (int i = 0; i < 0x80; i += 16) {
        printf("\r\n%02x:", i);
        for (int j = 0; j < 16; ++j)
            printf(" %02x", rf.readReg(i+j));
    }
    printf("\r\n");
}

void setFreq(uint32_t freq) {
    rf.writeReg(rf.REG_FRFMSB,   freq >> 16);
    rf.writeReg(rf.REG_FRFMSB+1, freq >> 8);
    rf.writeReg(rf.REG_FRFMSB+2, freq);
    //rf.writeReg(rf.REG_RXCONFIG, 0x28); // trigger RX restart not needed with FastHopOn
}

// sweepRadio performs one spectrum sweep from first by step for count steps. It stores the
// samples in the provided buffer in the form -2*rssi, e.g., for -109dBm it stores 238.
// The speed of the sweep is govered by usDelay, which is the number of microseconds to wait
// between setting the frequency and reading the RSSI value.
void sweepRadio(uint32_t first, uint32_t step, int count, uint8_t usDelay, uint8_t buf[]) {
    uint32_t freq = first;

    SysTick<72000000> delay;
    usDelay = usDelay >= 13 ? usDelay - 13 : 0;
    setFreq(freq);
    delay.micros(100*usDelay);
    for (int x = 0; x < count; ++x) {
        if (usDelay > 0) delay.micros(usDelay);
        // set next freq before reading RSSI, gains a few us and doesn't seem to
        // affect the RSSI that is in the pipeline...
        freq += step;
        setFreq(freq);
        // read RSSI
        *buf++ = rf.readReg(rf.REG_RSSIVALUE);
    }
    setFreq(first); // step back takes longer, so start now
}

void dumpRow(int count, uint8_t buf[]) {
    for (int x = 0; x < count; ++x) {
        printf(" %d", buf[x]);
    }
    printf("\r\n");
    wait_ms(10000);
}

// packetDump prints the scanline if a threshold is exceeded
void packetDump(int count, uint8_t buf[]) {
    // calculate some RSSI stats
    uint32_t sum=0, cnt=0;
    uint32_t max=0xff;
    for (int x = 0; x < count; ++x) {
        uint8_t rssi = buf[x];
        sum += (uint32_t)rssi;
        cnt++;
        if (rssi < max) max = rssi;
    }
    // dump if RSSI exceeds 80dBm
    if (max > 2*70) return;
    for (int x = 0; x < count; ++x) {
        printf(" %4d", -(int32_t)(buf[x]/2));
        if (x%20 == 19) printf("\r\n");
    }
    printf("\r\n");
    //wait_ms(500);
}


// statsDisplay is not functional yet...
void statsDisplay(int count, uint8_t buf[], uint32_t f0, uint32_t fStep) {
    int32_t sum=0, cnt=0;
    int32_t max=0xff;
    int maxIx=0;
    int32_t wSum=0, wCnt=0;
    for (int x = 0; x < count; ++x) {
        int32_t rssi = (uint32_t)buf[x];
        sum += rssi;
        cnt++;
        if (rssi < 2*75) {
            wSum += x * (2*75-rssi);
            wCnt += 2*75-rssi;
        }
        if (rssi < max) {
            max = rssi;
            maxIx = x;
        }
    }
    int32_t avg = sum/cnt;
    if (max > 2*70) return;
    int32_t ctr = f0+fStep*(wSum/wCnt);

    printf("PKT: %ddBm @%dHz, center %dHz, avg %ddBm\r\n", -max/2, f0+maxIx*fStep, ctr, -avg/2);
}

int main () {
#if 1
    fullSpeedClock(); // 72Mhz
#else
    enableSysTick();  // 8Mhz
#endif
    printf("\r\n===== Waterfall 2 starting...\r\n");

    // disable JTAG in AFIO-MAPR to release PB3, PB4, and PA15
    constexpr uint32_t afio = 0x40010000;
    MMIO32(afio + 0x04) |= 1 << 25; // disable JTAG, keep SWD enabled

    initLcd();
    //testPattern();
    lcd.freeze(16, 0);

    //printf("PB crl: 0x%08x\r\n", MMIO32(Periph::gpio+0x400+0));
    //printf("PB crh: 0x%08x\r\n", MMIO32(Periph::gpio+0x400+4));
    //printf("PB odr: 0x%08x\r\n", MMIO32(Periph::gpio+0x400+12));

    initPalette();

    initRadio(0);
    dumpRadioRegs();

    uint8_t interlace = 0;
    uint8_t bwConfig = 1;
    uint32_t center = 912500000;
    uint32_t middle = ((center<<2) / (32000000 >> 11)) << 6;

    //uint32_t step = bwConfigs[bwConfig].fdev;
    uint32_t step = 82;
    uint32_t first = middle - lcd.width/2 * step;
    uint32_t dF = lcd.width/2*step*61035/1000;
    //printf("middle: %d %x\r\n", middle, middle);

    for (int x=0; x<lcd.width; x+=lcd.width/4) {
        lcd.pixel(x, 12, 0xFFFF);
        lcd.pixel(x, 13, 0xFFFF);
        lcd.pixel(x, 14, 0xFFFF);
        lcd.pixel(x, 15, 0xFFFF);
        lcd_text.x = x;
        lcd_text.y = 1;
        lcd_printf("%dkHz", (center-dF+x*step*61035/1000)/1000);
    }

    initRadio(bwConfig);
    rf.setMode(rf.MODE_RECEIVE);
    wait_ms(10);

    while (true) {
        uint32_t start = ticks;

        printf("center %dkHz +/-%dkHz %d..%dkHz RxBW=%dHz step=%dHz\r\n",
            center / 1000, dF/1000, (center-dF)/1000, (center+dF)/1000,
            bwConfigs[bwConfig].fdev*61, step*61);

        static uint8_t rssiRow[lcd.width];

        for (int y = 16; y < lcd.height; ++y) {
            // sanity checks
            uint8_t mode = rf.readReg(rf.REG_OPMODE);
            if (mode != rf.MODE_RECEIVE) {
                printf("OOPS: mode=%02x\r\n", mode);
            }
            uint8_t irq1 = rf.readReg(rf.REG_IRQFLAGS1);
            if (irq1 != 0xd0) {
                printf("OOPS: irq1=%02x\r\n", irq1);
            }

            if (!interlace) {
              sweepRadio(first, step, lcd.width, bwConfigs[bwConfig].delay, rssiRow);
              //dumpRow(lcd.width, rssiRow);
            } else {
                static uint8_t rssiRow2[lcd.width/2], rssiRow3[lcd.width/2];
                sweepRadio(first,      2*step, lcd.width/2, bwConfigs[bwConfig].delay, rssiRow2);
                sweepRadio(first+step, 2*step, lcd.width/2, bwConfigs[bwConfig].delay, rssiRow3);
                for (int x=0; x<lcd.width/2; x++) {
                    rssiRow[2*x]   = rssiRow2[x];
                    rssiRow[2*x+1] = rssiRow3[x];
                }
            }
            sweepDisplay(y, lcd.width, rssiRow);
            //packetDump(lcd.width, rssiRow);
            statsDisplay(lcd.width, rssiRow, center-dF, step*61);
        }

        printf("screen=%dms sweep=%dms step=%dus\r\n",
            ticks - start, (ticks-start)/lcd.height, (ticks-start)*1000/lcd.height/lcd.width);

        //bwConfig = (bwConfig+interlace) % 2;
        //interlace = (interlace+1) %2;
        printf("bwConfig %d, interlace=%d\r\n", bwConfig, interlace);
    }
}
