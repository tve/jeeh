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

template< typename MO, typename MI, typename CK, typename SS >
struct RF96sa {
    void init (uint8_t bw=0, bool hf=true); // init radio with HF/LF and given bandwidth config
    void setMode (uint8_t newMode);
    void setFrequency (uint32_t freq);
    uint8_t rssi(); // returns rssi measurement in -dBm

    void sleep ();

    uint8_t readReg (uint8_t addr) {
        return spi.rwReg(addr, 0);
    }
    void writeReg (uint8_t addr, uint8_t val) {
        spi.rwReg(addr | 0x80, val);
    }

    enum {
        REG_FIFO          = 0x00,
        REG_OPMODE        = 0x01,
        REG_FRFMSB        = 0x07,
        REG_RSSIVALUE     = 0x11,
        REG_IRQFLAGS1     = 0x3e,
        REG_IRQFLAGS2     = 0x3f,

        MODE_SLEEP        = 0,
        MODE_STANDBY      = 1,
        MODE_FSRX         = 4,
        MODE_RECEIVE      = 5,

        IRQ1_MODEREADY    = 0x80,
    };

    void configure(const uint8_t* p); // configure given an array of regs/values

    SpiDev< MO, MI, CK, SS, 0 > spi;
};

// driver implementation

template< typename MO, typename MI, typename CK, typename SS >
void RF96sa<MO,MI,CK,SS>::setMode (uint8_t newMode) {
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xF8) | newMode);
    while ((readReg(REG_IRQFLAGS1) & IRQ1_MODEREADY) == 0) {
        //printf("Mode new:%02x now:%02x IRQ:%02x\r\n", newMode, readReg(REG_OPMODE), readReg(REG_IRQFLAGS1));
        //wait_ms(100);
        Yield();
    }
}

template< typename MO, typename MI, typename CK, typename SS >
void RF96sa<MO,MI,CK,SS>::setFrequency (uint32_t hz) {
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

template< typename MO, typename MI, typename CK, typename SS >
void RF96sa<MO,MI,CK,SS>::configure (const uint8_t* p) {
    while (true) {
        uint8_t cmd = p[0];
        if (cmd == 0)
            break;
        writeReg(cmd, p[1]);
        p += 2;
    }
}

static const uint8_t configRegs [] = {
    0x01, 0x00, // FSK mode, high-freq regs, sleep mode
    0x0b, 0x32, // Over-current protection @150mA
    0x0c, 0x20, // max LNA gain, no boost
    0x0d, 0x08, // AFC off, AGC on
    0x0e, 0x02, // 8-sample rssi smoothing
    0x10, 0x01, // unattainable rssi threshold: keep RX "seeking"
    0x1f, 0x00, // disable preamble detector: keep RX "seeking"
    0x27, 0x17, // sync size 8 bytes
    0x28, 0x2d, // random first sync byte to prevent matches
    0x44, 0xad, // enable fast hop
    0
};

// Step size and bandwidth configuration. n is the number of 61.035Hz oscillator
// steps to take for each scan step and bw is the mant/exp value for the RxBVW register.
static const struct { uint8_t bw; uint16_t br, fdev; } bwConfigs [] = {
    { 0x07, 6667,  82 }, // 80*61.035=4.8kHz step, 3.9kHz rxBW, 4.8kbps bit rate, 5kHz Fdev
    { 0x15, 3200, 164 }, // for 10kHz step: 10.4 RxBW (single-sided), 10kbps, 10kHz Fdev
};

template< typename MO, typename MI, typename CK, typename SS >
void RF96sa<MO,MI,CK,SS>::init (uint8_t bw, bool hf) {
    spi.init();
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

template< typename MO, typename MI, typename CK, typename SS >
void RF96sa<MO,MI,CK,SS>::sleep () {
    setMode(MODE_SLEEP);
}

template< typename MO, typename MI, typename CK, typename SS >
uint8_t RF96sa<MO,MI,CK,SS>::rssi () {
    return readReg(REG_RSSIVALUE)/2;
}
