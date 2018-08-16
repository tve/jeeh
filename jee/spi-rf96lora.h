// RFM96 / sx1276 LoRa driver
// This radio driver uses the LoRA modulation on the sx1276 to send and receive packets.

// The radio is connected via SPI and at this point the driver is primarily
// geared towards 433Mhz operation. Some minor changes need to be made to register
// settings and rssi calculations for operation in the 868Mhz or 915Mhz bands.

// The driver assumes that the antenna is connected to PA_BOOST, which is the case for the
// HopeRF rfm95 thru rfm98 modules, as well as the Dorji DRF1276 modules.

// The send function places a header as first payload byte into the packet. This header is used to
// address the destination and to control acknowledgments. It assumes a star network where all
// packets go to/from a central gateway.
// bit 7 ctrl: 0=data 1=ack/special
// bit 6 dest: 0=to-GW 1=from-GW
// bit 5 ack : 0=no-ack 1=ack-req
// bit 4 unused
// bits 0-3: 15 nodes, 0=broadcast
// c=0, a=0 : data, no ack requested
// c=0, a=1 : data, ack requested
// c=1, a=0 : ack
// c=1, a=1 : unused

// By convention, the byte after the header contains a packet type indentification byte as follows:
// bit 7 info: 0=no info, 1=last 2 bytes have rssi & fei
// bit 6 unused
// bit 0-5: 32 packet types

// Info trailer (present if packet type bit 7 is set):
// pkt[len-2]: 7-bit signed packet RX margin in dB, top bit always 0 (unused)
// pkt[len-1]: (fei_Hz + 64) / 128

// Packet type:
//  0: empty (may have info)
//  1: node info Vstart[mV], Vend[mV], Temp[cC], PktSent, PktRecv, ...?
//  2:

// Ack packets either consist of just the header byte or header plus info.

// Notes about RSSI and SNR:
// - SNR is not simply "packetRSSI - noiseRSSI", somehow LNA figures into it. At close proximity
//   the RSSI can be -80dB, noise -110dB yet SNR reported is in the 6-8dB range. So it says
//   something about what the radio used to "decode" the packet but does not provide information
//   about how much link budget is left/available.
// - SNR can be negative, that's what spread spectrum allows. SF7->-7.5dB, SF8->-10dB, SF12->-20dB
// - packet RSSI may be lower than noise RSSI, again, that's spread spectrum...
// - RX margin is calculated using the difference between SNR and the minimum SNR required to
//   demodulate a packet according to the datasheet.

#ifndef Yield
#define Yield()
#endif

// LoRaConfig captures a LoRa modem configuration, e.g. the ModemConfig1..3 registers.
typedef struct { uint8_t conf1; uint8_t conf2; uint8_t conf3; uint16_t bw; } LoRaConfig;

// Some "standard" configs follow. The names use bw: bandwidth in kHz, cr: coding rate 4/5..4/8,
// and sf: spreading factor.
//
// Configurations from radiohead library, the first one is fast for short range, the
// second intermediate for medium range, and the last two slow for long range.
LoRaConfig lora_bw500cr45sf7  = {0x92, 0x74, 0x04, 500}; // 31250bps, 20B in   14ms
LoRaConfig lora_bw125cr45sf7  = {0x72, 0x74, 0x04, 125}; //  7813bps, 20B in   57ms
LoRaConfig lora_bw125cr48sf12 = {0x78, 0xc4, 0x04, 125}; //   183bps, 20B in 1712ms
LoRaConfig lora_bw31cr48sf9   = {0x48, 0x94, 0x04,  31}; //   275bps, 20B in  987ms
// Configurations from LoRaWAN standard.
LoRaConfig lorawan_bw125sf12  = {0x72, 0xc4, 0x0C, 125}; //   250bps, 20B in 1319ms, -137dBm
LoRaConfig lorawan_bw125sf11  = {0x72, 0xb4, 0x0C, 125}; //   440bps, 20B in  660ms, -136dBm
LoRaConfig lorawan_bw125sf10  = {0x72, 0xa4, 0x04, 125}; //   980bps, 20B in  370ms, -134dBm
LoRaConfig lorawan_bw125sf9   = {0x72, 0x94, 0x04, 125}; //  1760bps, 20B in  185ms, -131dBm
LoRaConfig lorawan_bw125sf8   = {0x72, 0x84, 0x04, 125}; //  3125bps, 20B in  103ms, -128dBm
LoRaConfig lorawan_bw125sf7   = {0x72, 0x74, 0x04, 125}; //  5470bps, 20B in   57ms, -125dBm
LoRaConfig lorawan_bw250sf7   = {0x82, 0x74, 0x04, 250}; // 11000bps, 20B in   28ms, -122dBm
// Additional configurations all using CR47. The reason for CR47 is that CR45/CR46 can detect
// 1/2 bit errors, which doesn't improve link budget, just catches more errors. CR47 can correct
// single bit errors (1 bit wrong in a nibble)., whcih means it increases link budget. CR48 is
// like CR47 but can detect 2 bit errors, which again doesn't add to link budget.
LoRaConfig lora_bw125cr47sf7  = {0x76, 0x74, 0x04, 125}; //  4464bps, 20B in   71ms
LoRaConfig lora_bw125cr47sf8  = {0x76, 0x84, 0x04, 125}; //  2232bps, 20B in  128ms
LoRaConfig lora_bw125cr47sf9  = {0x76, 0x94, 0x04, 125}; //  1116bps, 20B in  226ms
LoRaConfig lora_bw125cr47sf10 = {0x76, 0xa4, 0x04, 125}; //   558bps, 20B in  453ms


template< typename SPI >
struct RF96lora {
    bool init (uint8_t id, uint8_t sync, LoRaConfig &conf, uint32_t freq);
    void frequency (uint32_t freq);
    void modemConfig (LoRaConfig &conf);
    void txPower (int8_t level);

    int receive (void* ptr, int len);
    void send (uint8_t header, const void* ptr, int len);
    int getAck(void* ptr, int len);
    int readAck(void* ptr, int len);
    void addInfo(uint8_t *ptr);
    void sleep (); // put the radio to sleep to save power
    void dumpRegs();
    int16_t noiseFloor(); // return the current RSSI as noise floor reading
    int16_t linkMargin(int16_t packetRssi, int16_t noiseFloor); // return link margin left in dB
    int16_t linkMargin(int8_t snr); // return link margin in dB

    uint8_t myId;

    // variables dealing with frequency and its correction
    uint32_t nomFreq; // nominal frequency
    uint32_t actFreq; // actual frequency programmed into device
    int32_t  fei;     // FEI of last packet received
    uint32_t bw;      // bandwidth of current config
    uint8_t  sf;      // spreading factor of current config
    int8_t   txpow;   // current tx power level in dBm

    // variables reporting info about the last packet received
    int16_t rssi;   // RSSI in dBm of last packet received
    int8_t  snr;    // SNR in dB of last packet received
    int8_t  margin; // signal margin in dB of last packet received, based on SNR
    uint8_t lna;    // LNA attenuation in dB

    uint8_t readReg (uint8_t addr) {
        return rwReg(addr, 0);
    }
    void writeReg (uint8_t addr, uint8_t val) {
        rwReg(addr | 0x80, val);
    }
    // TODO somewhat redundant to have readReg, writeReg, and rwReg ...
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
        REG_PACONFIG      = 0x09,
        REG_LNAVALUE      = 0x0C,
        REG_FIFOPTR       = 0x0D,
        REG_FIFORXCUR     = 0x10,
        REG_IRQFLAGS      = 0x12,
        REG_RXBYTES       = 0x13,
        REG_MODEMSTAT     = 0x18,
        REG_PKTSNR        = 0x19,
        REG_PKTRSSI       = 0x1A,
        REG_RSSIVALUE     = 0x1B,
        REG_MODEMCONF1    = 0x1D,
        REG_MODEMCONF2    = 0x1E,
        REG_PAYLENGTH     = 0x22,
        REG_MODEMCONF3    = 0x26,
        REG_PPMCORR       = 0x27,
        REG_FEI           = 0x28,
        REG_RSSIWIDE      = 0x2C,
        REG_SYNC          = 0x39,
        REG_DIOMAPPING1   = 0x40,
        REG_PADAC         = 0x4D,

        MODE_SLEEP        = 0,
        MODE_STANDBY      = 1,
        MODE_FSTX         = 2,
        MODE_TRANSMIT     = 3,
        MODE_RXCONT       = 5,
        MODE_RXSINGLE     = 6,

        IRQ_RXTIMEOUT    = 1<<7,
        IRQ_RXDONE       = 1<<6,
        IRQ_CRCERROR     = 1<<5,
        IRQ_VALIDHDR     = 1<<4,
        IRQ_TXDONE       = 1<<3,
        IRQ_CADDONE      = 1<<2,
        IRQ_FHSSCHG      = 1<<1,
        IRQ_CADDETECT    = 1<<0,
    };

    void setMode (uint8_t newMode);
    bool waitMode (uint8_t newMode);
    void configure (const uint8_t* p);
    void setFreq (uint32_t hz);
    void adjustFreq ();
    void adjustPow (uint8_t snr);
    void savePktInfo();
    int savePkt(void *ptr, int len);

    uint8_t mode;
};

// driver implementation

template< typename SPI >
void RF96lora<SPI>::setMode (uint8_t newMode) {
    newMode &= 7;
    mode = newMode;
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & ~0x7) | newMode);
}

template< typename SPI >
bool RF96lora<SPI>::waitMode (uint8_t newMode) {
    newMode &= 7;
    mode = newMode;
    newMode |= readReg(REG_OPMODE) & ~0x7;
    writeReg(REG_OPMODE, newMode);
    for (int i=0; i<100; i++) {
        if (readReg(REG_OPMODE) == newMode) return true;
        writeReg(REG_OPMODE, newMode);
        wait_ms(10);
    }
    return false;
}

// setFreq is an internal function, use setFrequency instead.
template< typename SPI >
void RF96lora<SPI>::setFreq (uint32_t hz) {
    // accept any frequency scale as input, including KHz and MHz
    // multiply by 10 until freq >= 100 MHz (don't specify 0 as input!)
    while (hz < 100000000)
        hz *= 10;
    actFreq = hz;

    // Couple of differences between HF and LF
    if (hz > 600000000) {
        writeReg(REG_OPMODE, (readReg(REG_OPMODE) & ~(1<<3))); // HF reg access
    } else {
        writeReg(REG_OPMODE, (readReg(REG_OPMODE) | (1<<3))); // LF reg access
    }

    // should be in standby to change freq
    uint8_t oldMode = mode;
    setMode(MODE_STANDBY);

    // Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz
    uint64_t frf = ((uint64_t)hz << 19) / (uint64_t)32000000;
    writeReg(REG_FRFMSB, frf>>16);
    writeReg(REG_FRFMSB+1, frf>>8);
    writeReg(REG_FRFMSB+2, frf);
    setMode(oldMode);
}

template< typename SPI >
void RF96lora<SPI>::frequency (uint32_t hz) {
    setFreq(hz);
    nomFreq = actFreq;
}

template< typename SPI >
void RF96lora<SPI>::modemConfig (LoRaConfig &conf) {
    uint8_t oldMode = mode;
    setMode(MODE_STANDBY);
    writeReg(REG_MODEMCONF1, conf.conf1 & ~1); // Force explicit header mode
    writeReg(REG_MODEMCONF2, (conf.conf2 & 0xf0) | 0x04); // Force TxSingle, CRC enable
    writeReg(REG_MODEMCONF3, conf.conf3 | 0x04); // Force LNA AGC
    bw = (uint32_t)conf.bw * 1000;
    sf = conf.conf2>>4;
    setMode(oldMode);
}

template< typename SPI >
int16_t RF96lora<SPI>::noiseFloor() {
    uint8_t oldMode = mode;
    setMode(MODE_RXCONT);
    wait_ms(3); // a bit of a guess, really depends on data rate?
    uint8_t rssi = readReg(REG_RSSIVALUE);
    setMode(oldMode);
    if (nomFreq > 600000000) return -157 + (int16_t)rssi;
    else return -164 + (int16_t)rssi;
}

static int8_t snr_margin[] = { 5, 7, 10, 12, 15, 17, 20 }; // dB below noise floor SF6..SF12

// linkMargin returns the dB of margin available given the rssi of a packet and the noise floor.
// According to datasheet table 13: range of spreading factors.
template< typename SPI >
int16_t RF96lora<SPI>::linkMargin(int16_t packetRssi, int16_t noiseFloor) {
    return packetRssi - (noiseFloor - (int16_t)snr_margin[sf-6]);
}

// linkMargin returns the dB of margin available given the snr of a packet.
// According to datasheet table 13: range of spreading factors.
template< typename SPI >
int16_t RF96lora<SPI>::linkMargin(int8_t snr) {
    return snr + (int16_t)snr_margin[sf-6];
}

template< typename SPI >
void RF96lora<SPI>::dumpRegs () {
    printf("   ");
    for (int i=0; i<16; i++) printf(" %02x", i);

    for (int i=0; i<0x71; i++) {
        if (i>=0x40 && i<0x60) continue;
        if (i%16==0) printf("\r\n%02x:", i);
        printf(" %02x", readReg(i));
    }
    printf("\r\n");
}

template< typename SPI >
void RF96lora<SPI>::configure (const uint8_t* p) {
    while (true) {
        uint8_t cmd = p[0];
        if (cmd == 0)
            break;
        writeReg(cmd, p[1]);
        p += 2;
    }
    mode = readReg(REG_OPMODE)&7;
}

// configRegs contains register-address, register-value pairs for initialization.
// Note that these are TvE's values, not JCW's, specifically, RxBW and Fdev have been optimized
// for rf69/rf96 and may not work with rf12's. Also, the sync bytes include one preamble byte to
// reduce the number of false sync word matches due to noise. These settings are compatible with the
// Go driver in https://github.com/tve/devices/tree/master/sx1231
static const uint8_t RF96LoRaConfig [] = {
    0x01, 0x89, // OpMode = LoRA+LF+standby
    0x09, 0xFF, // 17dBm output power
    0x0B, 0x32, // Over-current protection @150mA
    0x0C, 0xC3, // max LNA gain
    0x0D, 0x00, // FIFO ptr = 0
    0x0E, 0x00, // FIFO TX base = 0
    0x0F, 0x00, // FIFO RX base = 0
    0x10, 0x00, // FIFO RX current = 0
    0x11, 0x02, // mask FHSS change interrupt
    0x1f, 0x40, // rx-single timeout (0x40=64 symbol times)
    0x20, 0x00, // preamble of 8
    0x21, 0x08, //
    0x23, 0x42, // max payload of 66 bytes
    0x23, 0xff, // max payload of 255 bytes
    0x24, 0x00, // no freq hopping
    0x26, 0x04, // enable LNA AGC
    0x27, 0x00, // no ppm freq correction
    0x31, 0x03, // detection optimize for SF7-12
    0x33, 0x27, // no I/Q invert
    0x37, 0x0A, // detection threshold for SF7-12
    0x40, 0x00, // DIO mapping 1
    0x41, 0x00, // DIO mapping 2
    0x00 // sentinel
};

template< typename SPI >
bool RF96lora<SPI>::init (uint8_t id, uint8_t sync, LoRaConfig &conf, uint32_t freq) {
    myId = id;

    SPI::init();
    do
        writeReg(REG_DIOMAPPING1, 0xAA);
    while (readReg(REG_DIOMAPPING1) != 0xAA);
    do
        writeReg(REG_DIOMAPPING1, 0x55);
    while (readReg(REG_DIOMAPPING1) != 0x55);
    writeReg(REG_DIOMAPPING1, 0); // back to power-on default

    // put the chip into standby mode
    printf("sx1276 initial mode: %02x\r\n", readReg(REG_OPMODE));
    if (!waitMode(MODE_SLEEP)) {
        printf("sx1276 cannot sleep, mode: %02x\r\n", readReg(REG_OPMODE));
        return false;
    }
    writeReg(REG_OPMODE, 0x88); // switch to LoRa
    if (!waitMode(MODE_SLEEP)) {
        printf("sx1276 cannot sleep in LoRa, mode: %02x\r\n", readReg(REG_OPMODE));
        return false;
    }
    if (!waitMode(MODE_STANDBY)) {
        printf("sx1276 cannot standby, mode: %02x\r\n", readReg(REG_OPMODE));
        return false;
    }

    configure(RF96LoRaConfig);
    frequency(freq);
    modemConfig(conf);
    txpow = 17;

    writeReg(REG_SYNC, sync);

    dumpRegs();

    // make sure we can turn on the frequency synthesis (i.e. chip is not stuck)
    if (!waitMode(MODE_FSTX)) {
        printf("RF96lora: can't turn on PLL\r\n");
        return false;
    }
    setMode(MODE_STANDBY);
    return true;
}

// txPower sets the transmit power to the requested level in dB. The driver assumes that the
// PA_BOOST pin is used in the radio module and allows adjustment from 2dBm to 20dBm. The radio can
// go down to -4dBm(check?) but only when using the other output pin, which most modules don't seem
// to connect.
template< typename SPI >
void RF96lora<SPI>::txPower (int8_t level) {
    if (level < 2) level = 2;
    if (level > 20) level = 20;
    setMode(MODE_STANDBY);
    if (level > 17) {
        writeReg(REG_PADAC, 0x87); // turn 20dBm mode on
        writeReg(REG_PACONFIG, 0xf0+level-5);
    } else {
        writeReg(REG_PACONFIG, 0xf0+level-2);
        writeReg(REG_PADAC, 0x84); // turn 20dBm mode off
    }
    txpow = level;
}

template< typename SPI >
void RF96lora<SPI>::sleep () {
    setMode(MODE_SLEEP);
}

template< typename SPI >
void RF96lora<SPI>::adjustFreq () {
    int32_t corr = fei/4; // apply 1/4 of error as correction
    int32_t sbw = (int32_t)bw;
    if (corr > sbw/4) corr = sbw/4; // don't apply more than 1/4 of rx bandwidth
    if (corr < -sbw/4) corr = -sbw/4;
    setFreq(actFreq-corr); // apply correction
    int32_t deltaFreq = actFreq - nomFreq;
    int32_t ppmError = deltaFreq / (nomFreq/1000000);
    writeReg(REG_PPMCORR, ppmError);
}

// adjustPow changes the TX power based on the signal margin reported by the remote node in order to
// ensure that there is 10dB of margin. It adjusts the power downwards by approx 25% of what is
// theoretically needed to hit 10dB and it adjusts the power upwards by the full amt needed.
template< typename SPI >
void RF96lora<SPI>::adjustPow (uint8_t margin) {
    //int8_t txpow0 = txpow;
    if (margin > 14 && txpow > 2) {
        txPower(txpow - (margin-10+2)/4);
    } else if (margin > 10 && txpow > 2) {
        txPower(txpow-1);
    } else if (margin < 10 && txpow < 20) {
        txPower(txpow + 10-margin);
    }
    //if (txpow0 != txpow) printf("<TX {%d} %d->%ddB>", margin, txpow0, txpow);
}

static uint8_t RF96lnaMap[] = { 0, 0, 6, 12, 24, 36, 48, 48 };

// savePktInfo is an internal function to save the metadata for a received packet, such as RSSI,
// SNR, etc.
template< typename SPI >
void RF96lora<SPI>::savePktInfo() {
    // handle rssi, snr
    // Note bug in datasheet for RSSI calc, see official driver and
    // http://semtech.force.com/lora/lc_answers_questions?id=9064400000090EpAAI
    // https://github.com/Lora-net/LoRaMac-node/blob/master/src/radio/sx1276/sx1276.c#L1559
    snr = (int8_t)readReg(REG_PKTSNR)/4;
    rssi = nomFreq > 600000000 ? -157 : -164;
    uint8_t rawRssi = readReg(REG_PKTRSSI);
    rssi += rawRssi + (rawRssi>>4);
    if (snr < 0) rssi += snr;
    margin = linkMargin(snr);

    // handle LNA
    if (lna == 255) lna = 0; // we missed it, and 0 is the most likely value, sigh...

    // handle FEI
    int32_t f1 = (int32_t)readReg(REG_FEI) << 28 >> 12; // sign-extend
    f1 |= (int32_t)readReg(REG_FEI+1) << 8;
    f1 |= (int32_t)readReg(REG_FEI+2);
    //printf("FEI raw=%x bw=%d\r\n", f1, bw);
    fei = f1 * int32_t(bw/100) / 9537; // 9537=32Mhz*500/2^24/100
}

// savePkt is an internal function to save a received packet, incl. the metadata.
template< typename SPI >
int RF96lora<SPI>::savePkt(void *ptr, int len) {
    savePktInfo();
    writeReg(REG_FIFOPTR, readReg(REG_FIFORXCUR)); // set fifo ptr to start of pkt
    uint8_t count = readReg(REG_RXBYTES);
    int l = count;
    // empty FIFO
    SPI::enable();
    SPI::transfer(REG_FIFO);
    uint8_t *buf = (uint8_t*)ptr;
    while (count--) {
        uint8_t v = SPI::transfer(0);
        if (len-- > 0) *buf++ = v;
    }
    SPI::disable();
    writeReg(REG_IRQFLAGS, 0xff); // reset flags

    return l;
}

// receive initializes the radio for RX if it's not in RX mode, else it checks whether a
// packet is in the FIFO and pulls it out if it is. The returned value is -1 if there is no
// packet, else the length of the packet. The fei, rssi, and lna values are also
// valid when a packet has been received but may change with the next call to receive().
template< typename SPI >
int RF96lora<SPI>::receive (void* ptr, int len) {
    int ret = -1;
    uint8_t irqfl = readReg(REG_IRQFLAGS);
    if ((irqfl & IRQ_RXDONE) != 0) {
        ret = savePkt(ptr, len);
        setMode(MODE_RXCONT);
    } else if ((irqfl & (IRQ_VALIDHDR|IRQ_RXDONE)) == IRQ_VALIDHDR && lna == 255) {
        lna = RF96lnaMap[ (readReg(REG_LNAVALUE) >> 5) & 0x7 ];
    } else if ((readReg(REG_OPMODE) & 0x7) != MODE_RXCONT) {
        setMode(MODE_RXCONT);
    }
    return ret;
}

// readAck assumes that a packet is ready in the FIFO, reads it and processes the FEI and SNR
// info it carries in the first two bytes to adjust TX power and frequency. It copies the packet
// to the provided buffer and returns its length.
template< typename SPI >
int RF96lora<SPI>::readAck(void* ptr, int len) {
    int l = savePkt(ptr, len);
    uint8_t *buf = (uint8_t*)ptr; // get a pointer we can dereference
    if ((buf[0] & 0xE0) == 0xC0 && l > 2) {
        // it's an ACK from GW
        adjustFreq(); // adjust based on what we measured, not what GW says...
#if ADJPOW
        if ((buf[1] & 0x80) != 0) {
            adjustPow(buf[l-2]);
        }
#endif
    }
    return l;
}

// getAck briefly turns on the receiver to get an ACK to a just-transmitted packet. It returns the
// number of bytes received, or -1 if more waiting is needed, or 0 if the ack wait timed out.
// If an ack is receivced and it carries FEI and SNR info then adjustPowFreq is called.
template< typename SPI >
int RF96lora<SPI>::getAck(void* ptr, int len) {
    uint8_t mode = readReg(REG_OPMODE) & 0x7;
    if (mode == MODE_FSTX || mode == MODE_TRANSMIT) return -1; // need to wait...

    uint8_t irqFlags = readReg(REG_IRQFLAGS);
    //printf("{%d,%02x}", mode, irqFlags);

    // Capture LNA. We're using AGC but the chip does set REG_LNAVALUE to the currently chosen gain.
    // The catch is that this varies in real-time and at the end of the reception the LNA always
    // goes back to max gain, so by the time IRQ_RXDONE comes around the real LNA gain setting is
    // lost. The best we can do is to capture
    if (mode == MODE_RXSINGLE) {
        if ((irqFlags & (IRQ_VALIDHDR|IRQ_RXDONE)) == IRQ_VALIDHDR && lna == 255) {
            lna = RF96lnaMap[ (readReg(REG_LNAVALUE) >> 5) & 0x7 ];
            //printf("<LNA%d>", lna);
        }
        return -1; // need to wait...
    }

    if ((irqFlags & IRQ_TXDONE) != 0) {
        // just finished TX, need to switch to RX_SINGLE
        writeReg(REG_IRQFLAGS, 0xff); // clear all flags
        setMode(MODE_RXSINGLE);
        lna = 255; // set to invalid value

        return -1;
    }

    if ((irqFlags & IRQ_RXDONE) != 0) {
        // got ack!
        return readAck(ptr, len);
    }

    if ((irqFlags & IRQ_RXTIMEOUT) != 0) {
        // ack wait timed out :-(
#if ADJPOW
        if (txpow < 20) txPower(txpow+1);
#endif
        return 0;
    }

    // we land here at the end of a transmission: REG_OPMODE changes to standby before
    // REG_IRQFLAGS shows TXDONE (ouch)! There may be other situations and we may lock-up, it's
    // a bit unknown right now...
    return -1;
}

// Add 2 bytes of info at ptr[0] and ptr[1] to the outgoing packet to signal margin and FEI to the
// other party. Note that bit 7 in packet type byte needs to be set too!
template< typename SPI >
void RF96lora<SPI>::addInfo(uint8_t *ptr) {
    int8_t m = margin;
    if (m > 63) m = 63;
    if (m < 0) m = 0;
    ptr[0] = m;
    ptr[1] = (fei+64) >> 7;
}

// send transmits the packet as specified by the header.
template< typename SPI >
void RF96lora<SPI>::send (uint8_t header, const void* ptr, int len) {
    setMode(MODE_FSTX);
    writeReg(REG_IRQFLAGS, 0xff); // clear all flags
    writeReg(REG_FIFOPTR, 0);
    writeReg(REG_PAYLENGTH, len+1); // length incl header byte

    SPI::enable();
    SPI::transfer(REG_FIFO | 0x80);
    SPI::transfer(header);
    for (uint8_t i=0; i<len; ++i)
        SPI::transfer(((const uint8_t*) ptr)[i]);
    SPI::disable();

    setMode(MODE_TRANSMIT);
#if 0
    uint8_t mode = readReg(REG_OPMODE)&0x7;
    //printf("sx1276 mode: %02x\r\n", readReg(REG_OPMODE));
    if (mode != MODE_FSTX && mode != MODE_TRANSMIT)
      printf("sx1276 mode: %d (expected FSTX or TX) [was %d]\r\n", mode, mode0);
    while ((readReg(REG_IRQFLAGS) & IRQ_TXDONE) == 0)
        Yield();

    setMode(MODE_STANDBY);
#endif
}
