template< typename SPI >
struct SdCard {
    constexpr static auto TIMEOUT = 50'000; // arbitrary

    SPI& spi;
    bool sdhc;

    SdCard (SPI& s) : spi (s) {}

    void init () {
        for (int i = 0; i < 10; ++i)
            spi.rwByte(0xFF);

        auto r = cmd(0, 0, 0x95);
        logf("c0 %02x", r);

        r = cmd(8, 0x1AA, 0x87);
        logf("c8 %02x %08x", r, get32());

        cmdRep(5541, 1<<30);
        cmdRep(5541, 0);

        cmdRep(58, 0);
        auto v = get32();
        logf("  %08x", v);

        cmdRep(16, 512);
        spi.disable();

        sdhc = (v >> 30) & 1;
        logf("sdhc %d", sdhc);
    }

    int read (uint32_t page, uint8_t* buf) const {
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

    int write (uint32_t page, uint8_t const* buf) const {
        cmd(24, sdhc ? page : page * 512);
        spi.rwByte(0xFF);
        spi.rwByte(0xFE);
        for (int i = 0; i < 512; ++i)
            spi.rwByte(*buf++);
        spi.rwByte(0xFF);
        spi.disable();
        return 512;
    }

private:
    void cmdRep (int req, uint32_t arg) const {
        int r;
        do {
            if (req >= 100)
                cmd(req/100, 0);
            r = cmd(req % 100, arg);
        } while (r == 1);
        logf("c%d-%d %02x", req, arg != 0, r);
    }

    int cmd (int req, uint32_t arg, uint8_t crc =0) const {
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

    uint32_t get32 () const {
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i)
            v = (v<<8) | spi.rwByte(0xFF);
        return v;
    }
};

template< typename BLK >
struct FatFS {
    BLK& blk;

    FatFS (BLK& s) : blk (s) {}

    void init () {
        blk.read(0, buf);                        // find boot sector
        base = *(uint32_t*) (buf+0x1C6);         // base for everything

        blk.read(base, buf);                     // location of boot rec
        spc = buf[0x0D];                         // sectors per cluster
        rsec = (uint16_t&) buf[0x0E];            // reserved sectors
        uint8_t nfc = buf[0x10];                 // number of FAT copies
        uint16_t spf = *(uint16_t*) (buf+0x16);  // sectors per fat
        rdir = nfc * spf + rsec + base;          // location of root dir
        rmax = buf[0x11] | buf[0x12]<<8;         // max root entries
        data = (rmax >> 4) + rdir;               // start of data area
        uint32_t tsc = buf[0x13] | buf[0x14]<<8; // total sector count
        if (tsc == 0)
            tsc = (uint32_t&) buf[0x20];         // ... or get 32-bit count
        clim = tsc / spc + 1;                    // cluster limit
#if 1
        logf("b %d spc %d rs %d nfc %d spf %d  rd %d rm %d da %d tsc %d cl %d",
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
            blk.read(base + rsec + curr, buf);
        }

        if (clim >= 4096)  // is it FAT16?
            return *(uint16_t*) (buf + off % 512);

        // TODO untested:
        // 12-bit entries need special care, as they may span across sectors

        if (cn & 1)
            ++off;

        uint8_t b1 = buf[off];
        off = (off+1) % 512;
        if (off == 0)
            blk.read(base + rsec + ++curr, buf);
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

template< typename FS >
struct FileMap {
    enum { NFRAG = 3 };
    uint16_t map [NFRAG] {};
    uint8_t size [NFRAG] {};
    FS& fs;

    FileMap (FS& fat) : fs (fat) {}

    int open (char const* name) {
        char fnBuf [11];
        conv83(name, fnBuf);

        for (auto i = 0; i < fs.rmax; ++i) {
            int off = (i*32) % 512;
            if (off == 0)
                fs.blk.read(fs.rdir + i/16, fs.buf);
            if (memcmp(fnBuf, fs.buf + off, sizeof fnBuf) == 0) {
                auto bytes = (uint32_t&) fs.buf[off+28];
                fs.curr = ~0; // consider buf to be empty at this point

                auto n = 0;
                auto cluster = (uint16_t&) fs.buf[off+26];
                while (2 <= cluster && cluster < fs.clim) {
                    while (n < NFRAG) {
                        if (size[n] == 0)
                            map[n] = cluster;
                        if (cluster == map[n] + size[n]) {
                            ++size[n];
                            break;
                        }
                        if (++n >= NFRAG)
                            return -2; // too many fragments
                    }
                    cluster = fs.chain(cluster);
                }
                return bytes;
            }
        }
        return -1; // not found
    }

    static void conv83 (char const* name, char fnBuf [11]) {
        memset(fnBuf, ' ', 11);
        for (auto i = 0U; *name != 0; ++name)
            if (auto c = *name; c == '.')
                i = 8;
            else if (i < 11)
                fnBuf[i++] = c - ('a' <= c && c <= 'z' ? 0x20 : 0);
    }

    bool rwBlock (bool wr, int num, void* buf) const {
        uint16_t grp = num / fs.spc;
        int i = 0;
        while (grp >= size[i]) {
            grp -= size[i];
            if (++i >= NFRAG)
                return false;
        }
        uint16_t off = fs.data + (map[i] + grp - 2) * fs.spc + num % fs.spc;
        logf("rwBlock(%d,%d) => %d", wr, num, off);
        if (wr)
            fs.blk.write(off, (uint8_t const*) buf);
        else
            fs.blk.read(off, (uint8_t*) buf);
        return true;
    }
};
