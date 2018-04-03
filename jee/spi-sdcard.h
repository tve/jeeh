// Driver for SD card access, supports both SD (v1) and SDHD (v2).
//
// Some cards may not work, this does not use slow SPI access during init.

template< typename MO, typename MI, typename CK, typename SS >
struct SdCard {
    static int init () {
        // try *without* and then *with* HCS bit 30 set
        // this determines whether it's an SD (v1) or an SDHD (v2) card
        for (info.type = 0; info.type < 2; ++info.type) {
            // reset SPI register to known state
            spi.disable();
            for (int i = 0; i < 10; ++i)
                spi.transfer(0xFF);

            bool ok = false;
            for (int i = 0; !ok && i < 100; ++i)
                ok = cmd(0, 0) == 0x01;  // wait for IDLE
            if (!ok)
                continue;  // no response, probably no card present

            cmd(8, 0x000001AA, 0x87); wait();

            for (int i = 0; i < 15; ++i) {
                cmd(55, 0); wait();
                if (cmd(41, info.type << 30) == 0)
                    return info.type;  // 0 = SD, 1 = SDHD
            }
        }

        return info.type = -1;  // no valid card found
    }

    // TODO move elsewhere, it does not belong in this low-level block driver
    static void fat16 () {
        read512(0);                                      // find boot sector
        info.base = *(uint32_t*) (info.buf + 0x1C6);     // base for everything

        read512(info.base);                              // location of boot rec
        info.spc = info.buf[0x0D];                       // sectors per cluster
        uint16_t rsec = *(uint16_t*) (info.buf + 0x0E);  // reserved sectors
        uint8_t nfc = info.buf[0x10];                    // number of FAT copies
        uint16_t spf = *(uint16_t*) (info.buf + 0x16);   // sectors per fat
        info.rdir = nfc * spf + rsec + info.base;        // location of root dir
        info.rmax = *(uint16_t*) (info.buf + 0x11);      // max root entries
        info.data = (info.rmax >> 4) + info.rdir;        // start of data area

      //printf("base %d spc %d rsec %d nfc %d spf %d rdir %d rmax %d data %d\n",
      //  info.base, info.spc, rsec, nfc, spf, info.rdir, info.rmax, info.data);
    }

    static void read512 (int page) {
        if (info.type == 0)
            page <<= 9;
        int last = cmd(17, page);
        while (last != 0xFE)
            last = spi.transfer(0xFF);
        for (int i = 0; i < 512; ++i)
            info.buf[i] = spi.transfer(0xFF);
        send16b(0xFFFF);
        wait();
    }

    static void write512 (int page) {
        if (info.type == 0)
            page <<= 9;
        int r = cmd(24, page);
        send16b(0xFFFE);
        for (int i = 0; i < 512; ++i)
            spi.transfer(info.buf[i]);
        send16b(0xFFFF);
        wait();
    }

    static void send16b (uint16_t v) {
        spi.transfer(v >> 8);
        spi.transfer(v);
    }

    static int cmd (int req, uint32_t arg, uint8_t crc =0x95) {
        spi.disable();

        spi.enable();
        send16b(0xFF40 | req);
        send16b(arg >> 16);
        send16b(arg);
        spi.transfer(crc);

        for (;;) {
            int r = spi.transfer(0xFF);
            if ((r & 0x80) == 0)
                return r;
        }
    }

    static void wait () {
        while (spi.transfer(0xFF) != 0xFF) ;
        spi.disable();
    }

    static SpiDev<MO,MI,CK,SS> spi;

    // TODO move elsewhere, it does not belong in this low-level block driver
    static struct FatInfo {
        uint32_t base;      // base sector for everything
        uint32_t rdir;      // location of root dir
        uint32_t data;      // start sector of data area
        uint16_t rmax;      // max root entries
        uint8_t spc;        // sectors per cluster
        int8_t type;        // -1 = invalid, 0 = SD, 1 = SDHD
        uint8_t buf [512];  // buffer space for one sector
    } info;
};

template< typename MO, typename MI, typename CK, typename SS >
SpiDev<MO,MI,CK,SS> SdCard<MO,MI,CK,SS>::spi;

template< typename MO, typename MI, typename CK, typename SS >
struct SdCard<MO,MI,CK,SS>::FatInfo SdCard<MO,MI,CK,SS>::info;

// SpiDev< PinB<5>, PinB<4>, PinB<3>, PinA<15> > sd;
typedef SlowPin< PinA<15>, 10 > SdSel;
SdCard< PinB<5>, PinB<4>, PinB<3>, PinA<15> > sd;
