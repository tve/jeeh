template< typename SPI >
struct SdCard {
    constexpr static auto TIMEOUT = 50'000; // arbitrary

    SPI& spi;

    SdCard (SPI& s) : spi (s) {}

    void init () {
        for (int i = 0; i < 10; ++i)
            spi.rwByte(0xFF);

        auto r = cmd(0, 0, 0x95);
        logf("c0 %d", r);

        r = cmd(8, 0x1AA, 0x87);
        logf("c8 %d %08x", r, get32());

        do {
            cmd(55, 0);
            r = cmd(41, 1<<30);
        } while (r == 1);
        logf("c41-1 %d", r);

        do {
            cmd(55, 0);
            r = cmd(41, 0);
        } while (r == 1);
        logf("c41-0 %d", r);

        do {
            r = cmd(58, 0);
        } while (r == 1);
        auto v = get32();
        logf("c58 %d %08x", r, v);
        sdhc = (v & (1<<30)) != 0;

        do {
            r = cmd(16, 512);
        } while (r == 1);
        logf("c16 %d", r);

        spi.disable();
    }

    auto readBlock (uint32_t page, uint8_t* buf) const -> int {
        int last = cmd(17, sdhc ? page : page * 512);
        for (int i = 0; last != 0xFE; ++i) {
            if (++i >= TIMEOUT)
                return 0;
            last = spi.rwByte(0xFF);
        }
        for (int i = 0; i < 512; ++i)
            *buf++ = spi.rwByte(0xFF);
        spi.rwByte(0xFF);
        spi.disable();
        return 512;
    }

    auto writeBlock (uint32_t page, uint8_t const* buf) const -> int {
        cmd(24, sdhc ? page : page * 512);
        spi.rwByte(0xFF);
        spi.rwByte(0xFE);
        for (int i = 0; i < 512; ++i)
            spi.rwByte(*buf++);
        spi.rwByte(0xFF);
        spi.disable();
        return 512;
    }

    bool sdhc =false;
private:
    auto cmd (int req, uint32_t arg, uint8_t crc =0) const -> int {
        spi.disable();
        spi.enable();
        wait();

        spi.rwByte(0x40 | req);
        spi.rwByte(arg >> 24);
        spi.rwByte(arg >> 16);
        spi.rwByte(arg >> 8);
        spi.rwByte(arg);
        spi.rwByte(crc);

        for (int i = 0; i < 1000; ++i)
            if (uint8_t r = spi.rwByte(0xFF); r != 0xFF)
                return r;

        return -1;
    }

    void wait () const {
        for (int i = 0; i < TIMEOUT; ++i)
            if (spi.rwByte(0xFF) == 0xFF)
                return;
    }

    auto get32 () const -> uint32_t {
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i)
            v = (v<<8) | spi.rwByte(0xFF);
        return v;
    }
};

template< typename T >
struct FatFS {
    T& sd;

    FatFS (T& s) : sd (s) {}

    void init () {
        sd.readBlock(0, buf);                       // find boot sector
        base = *(uint32_t*) (buf+0x1C6);          // base for everything

        sd.readBlock(base, buf);                    // location of boot rec
        spc = buf[0x0D];                          // sectors per cluster
        rsec = *(uint16_t*) (buf+0x0E);           // reserved sectors
        uint8_t nfc = buf[0x10];                  // number of FAT copies
        uint16_t spf = *(uint16_t*) (buf+0x16);   // sectors per fat
        rdir = nfc * spf + rsec + base;           // location of root dir
        rmax = buf[0x11] | buf[0x12]<<8;          // max root entries
        data = (rmax >> 4) + rdir;                // start of data area
        uint32_t tsc = buf[0x13] | buf[0x14]<<8;  // total sector count
        if (tsc == 0)
            tsc = *(uint32_t*) (buf+0x20);        // ... or get 32-bit count
        clim = tsc / spc + 1;                     // cluster limit
#if 0
        logf("base %d spc %d rsec %d nfc %d spf %d"
             " rdir %d rmax %d data %d tsc %d clim %d",
            base, spc, rsec, nfc, spf, rdir, rmax, data, tsc, clim);
#endif
    }

    // TODO use buf[] to read on-demand if fat sector is not in memory
    int chain (int cn) {
        if (cn < 2 || cn >= clim)
            return 0;

        int off = clim < 4096 ? cn/2*3 : cn*2;  // 12 or 16 bits per entry
        if (curr != off/512) {
            curr = off/512;
            sd.readBlock(base + rsec + curr, buf);
        }

        if (clim >= 4096)  // is it FAT16?
            return *(uint16_t*) (buf + off % 512);

        // TODO untested:
        // 12-bit entries needs special care, as they may span across sectors

        if (cn & 1)
            ++off;

        uint8_t b1 = buf[off];
        off = (off+1) % 512;
        if (off == 0)
            sd.readBlock(base + rsec + ++curr, buf);
        uint8_t b2 = buf[off];

        return cn & 1 ? b1>>4 | b2<<4 : b1 | (b2&0xF)<<8;
    }

    uint32_t base;          // base sector for everything
    uint32_t rdir;          // location of root dir
    uint32_t data;          // start sector of data area
    uint16_t rmax;          // max root entries
    uint16_t rsec;          // reserved sectors
    uint16_t clim;          // cluster limit
    uint8_t spc;            // sectors per cluster

    uint16_t curr;          // current sector in buffer (during chain calls)
    uint8_t buf [512];      // buffer space for one sector
};

template< typename T, int N >
struct FileMap {
    FileMap (T& f) : fat (f) {
        memset(map, 0, sizeof map);
    }

    int open (char const name [11]) {
        for (int i = 0; i < fat.rmax; ++i) {
            int off = (i*32) % 512;
            if (off == 0)
                fat.sd.readBlock(fat.rdir + i/16, fat.buf);
            if (memcmp(name, fat.buf + off, 11) == 0) {
                int cluster = *(uint16_t*) (fat.buf + off + 26);
                int length = *(uint32_t*) (fat.buf + off + 28);
                //for (int j = 0; j < 11; ++j) {
                //    if (j == 8)
                //        printf(".");
                //    printf("%c", name[j]);
                //}
                fat.curr = ~0; // consider buf to be empty at this point
                int n = 0;
                while (2 <= cluster && cluster < fat.clim) {
                    printf("%d,", cluster);
                    map[n++] = cluster;
                    cluster = fat.chain(cluster);
                }
                printf(" %d @ %d, %db\n", n, cluster, length);
                return length;
            }
        }
        return -1;
    }

    bool ioSect (bool wr, int num, void* buf) {
        uint16_t grp = num / fat.spc;
        if (grp >= N || map[grp] == 0)
            return false;
        uint16_t off = fat.data + (map[grp] - 2) * fat.spc + num % fat.spc;
        logf("rwSect(%d,%d) => %d", wr, num, off);
        if (wr)
            fat.sd.writeBlock(off, buf);
        else
            fat.sd.readBlock(off, buf);
        return true;
    }

    uint16_t map [N];
    T& fat;
};
