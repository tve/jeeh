// Native mode RF69 driver.
//
// JeeLabs packet format:
//  0: packet length N = 2..65
//  1: group parity (b7..6), dest id (b5..0)
//  2: flags (b7..6), src id (b5..0)
//  3..(N-1): payload data (max 63 bytes)
//  N..(N+1): 16-bit crc

#ifndef Yield
#define Yield()
#endif

template< typename SPI >
struct RF96sa {
    void init (uint8_t bw=0, bool hf=true); // init radio with HF/LF and given bandwidth config
    void setMode (uint8_t newMode);
    void setFrequency (uint32_t freq);
    uint8_t rssi(); // returns rssi measurement in -dBm

    void sleep ();

    uint8_t readReg (uint8_t addr) {
        return rwReg(addr, 0);
    }
    void writeReg (uint8_t addr, uint8_t val) {
        rwReg(addr | 0x80, val);
    }
    static uint8_t rwReg (uint8_t cmd, uint8_t val) {
        SPI::enable();
        SPI::transfer(cmd);
        uint8_t r = SPI::transfer(val);
        SPI::disable();
        return r;
    }

    enum {
        REG_FIFO          = 0x00,
        REG_OPMODE        = 0x01,
        REG_FRFMSB        = 0x06,
        REG_RSSIVALUE     = 0x11,
        REG_RXCONFIG      = 0x28,
        REG_IRQFLAGS1     = 0x3e,
        REG_IRQFLAGS2     = 0x3f,

        MODE_SLEEP        = 0,
        MODE_STANDBY      = 1,
        MODE_FSRX         = 4,
        MODE_RECEIVE      = 5,

        IRQ1_MODEREADY    = 0x80,
    };

    void configure(const uint8_t* p); // configure given an array of regs/values
};

// driver implementation

template< typename SPI >
void RF96sa<SPI>::setMode (uint8_t newMode) {
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xF8) | newMode);
    while ((readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) == 0) {
        //printf("Mode new:%02x now:%02x IRQ:%02x\r\n", newMode, readReg(REG_OPMODE), readReg(REG_IRQFLAGS1));
        //wait_ms(100);
        Yield();
    }
}

template< typename SPI >
void RF96sa<SPI>::setFrequency (uint32_t hz) {
    // accept any frequency scale as input, including KHz and MHz
    // multiply by 10 until freq >= 100 MHz (don't specify 0 as input!)
    while (hz < 100000000)
        hz *= 10;

    // Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz
    // use multiples of 64 to avoid multi-precision arithmetic, i.e. 3906.25 Hz
    // due to this, the lower 6 bits of the calculated factor will always be 0
    // this is still 4 ppm, i.e. well below the radio's 32 MHz crystal accuracy
    // 868.0 MHz = 0xD90000, 868.3 MHz = 0xD91300, 915.0 MHz = 0xE4C000
    uint32_t frf = (hz << 2) / (32000000L >> 11);
    writeReg(REG_FRFMSB, frf >> 10);
    writeReg(REG_FRFMSB+1, frf >> 2);
    writeReg(REG_FRFMSB+2, frf << 6);
}

template< typename SPI >
void RF96sa<SPI>::configure (const uint8_t* p) {
    while (true) {
        uint8_t cmd = p[0];
        if (cmd == 0)
            break;
        writeReg(cmd, p[1]);
        p += 2;
    }
}

static const uint8_t configRegs [] = {
    0x01, 0x20, // FSK mode, high-freq regs, sleep mode
    0x01, 0x20, // FSK mode, high-freq regs, sleep mode
    0x0b, 0x32, // Over-current protection @150mA
    0x0c, 0x20, // max LNA gain, no boost
    0x0d, 0x08, // AFC off, AGC on
    //0x0e, 0x02, // 8-sample rssi smoothing
    0x0e, 0x00, // 2-sample rssi smoothing
    0x10, 0x01, // unattainable rssi threshold: keep RX "seeking"
    0x1f, 0x00, // disable preamble detector: keep RX "seeking"
    0x27, 0x17, // sync size 8 bytes
    0x28, 0x2d, // random first sync byte to prevent matches
    0x44, 0xad, // enable fast hop
    0
};

// The RSSI sampling rate in the sx1276 is determined by the RxBW setting. The formula for the
// update rate in milliseconds is oversampling/4*RxBW, where oversampling is the smothing value
// configured in RegRssiConfig (minimum of 2) and RxBW is in kHz. E.g. for RxBw=10.4kHz and
// smoothing=2 the updates of RegRssiValue happen every 48us.
// bw is the mant/exp value for the RxBW register, delay the time for RSSI to settle,
// br and fDev are the bit-rate and freq dev settings that go with it although they may have no
// impact...
const struct { uint8_t bw, delay; uint16_t br, fdev; } bwConfigs [] = {
    { 0x15, 48, 3200, 164 }, // for 10kHz step: 10.4 RxBW (single-sided), 48us, 10kbps, 10kHz Fdev
    { 0x14, 24, 1600, 328 }, // for 20kHz step: 20.8 RxBW (single-sided), 24us, 20kbps, 20kHz Fdev
};

// 10.4kHz RxBW, 48us/step: 11.5ms per sweep, 10kHz/step: 2.4Mhz
// 20.8kHz RxBW, 24us/step: 5.77ms per sweep

template< typename SPI >
void RF96sa<SPI>::init (uint8_t bw, bool hf) {
    configure(configRegs);
    writeReg(REG_OPMODE, readReg(REG_OPMODE) & (!hf << 3)); // set/clear HF mode
    auto c = bwConfigs[bw];
    writeReg(0x12, c.bw); // set RxBW
    writeReg(0x13, c.bw); // set AfcBW
    writeReg(0x02, c.br>>8); // set bit rate MSB
    writeReg(0x03, c.br);    // set bit rate LSB
    writeReg(0x04, c.fdev>>8); // set Fdev MSB
    writeReg(0x05, c.fdev);    // set Fdev LSB
}

template< typename SPI >
void RF96sa<SPI>::sleep () {
    setMode(MODE_SLEEP);
}

template< typename SPI >
uint8_t RF96sa<SPI>::rssi () {
    return readReg(REG_RSSIVALUE)/2;
}
