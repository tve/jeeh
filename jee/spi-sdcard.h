// Driver for SD card access, supports both SD (v1) and SDHC (v2).
//
// Some cards may not work, this does not use slow SPI access during init.

template< typename MO, typename MI, typename CK, typename SS >
struct SdCard {
    static bool init () {
        // try *without* and then *with* HCS bit 30 set
        // this determines whether it's an SD (v1) or an SDHC (v2) card
        for (sdhc = 0; sdhc < 2; ++sdhc) {
            // reset SPI register to known state
            spi.disable();
            for (int i = 0; i < 10; ++i)
                spi.transfer(0xFF);

            bool ok = false;
            for (int i = 0; !ok && i < 100; ++i)
                ok = cmd(0, 0, 0x95) == 0x01;  // wait for IDLE
            if (!ok)
                continue;  // no response, probably no card present

            cmd(8, 0x000001AA, 0x87); wait();  // required by SDHC cards

            for (int i = 0; i < 15; ++i) {
                cmd(55, 0); wait();
                if (cmd(41, sdhc << 30) == 0)
                    return true;
            }
        }
        return false;  // no valid card found
    }

    static void read512 (int page, void* buf) {
        int last = cmd(17, sdhc ? page : page << 9);
        while (last != 0xFE)
            last = spi.transfer(0xFF);
        for (int i = 0; i < 512; ++i)
            ((uint8_t*) buf)[i] = spi.transfer(0xFF);
        send16b(0xFFFF);
        wait();
    }

    static void write512 (int page, uint8_t const* buf) {
        cmd(24, sdhc ? page : page << 9);
        send16b(0xFFFE);
        for (int i = 0; i < 512; ++i)
            spi.transfer(((uint8_t const*) buf)[i]);
        send16b(0xFFFF);
        wait();
    }

    static void send16b (uint16_t v) {
        spi.transfer(v >> 8);
        spi.transfer(v);
    }

    static int cmd (int req, uint32_t arg, uint8_t crc =0) {
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
    static uint8_t sdhc; // 0 = SD, 1 = SDHC
};

template< typename MO, typename MI, typename CK, typename SS >
SpiDev<MO,MI,CK,SS> SdCard<MO,MI,CK,SS>::spi;

template< typename MO, typename MI, typename CK, typename SS >
uint8_t SdCard<MO,MI,CK,SS>::sdhc;
