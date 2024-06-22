// Quad SPI for the 64 MBit flash on Disco-F723

namespace qspi {
using namespace jeeh;

// see ST's ref man: RM0431 rev 3, section 13, p.347 (QUADSPI)
enum { CR=0x00, DCR=0x04, SR=0x08, DLR=0x10, CCR=0x14, AR=0x18, DR=0x20,
        PSMKR=0x24, PSMAR=0x28, PIR=0x2C, LPTR=0x30 };

const auto addr = (uint32_t const*) 0x9000'0000;
constexpr auto fsize = 26; // flash has 64 MB, 2^26 bytes
constexpr auto dummy = 6;  // number of cycles between cmd and data

void waitBusy () {
    while (QUADSPI[SR](5)) {} // wait until not busy
}

void mmapEnable () {
    waitBusy();
    QUADSPI[DLR] = 0;
    QUADSPI[PIR] = 0x10;
    QUADSPI[PSMKR] = 1<<0;
    QUADSPI[PSMAR] = 0;
    // mem-mapped: FMODE DMODE, IMODE INS
    QUADSPI[CCR] = (2<<26) | (3<<24) | (3<<8) | (0x05<<0); // poll status

    waitBusy();
    // mem-mapped: DDRM DHHC FMODE DMODE DCYC, ADSIZE ADMODE IMODE INS
    QUADSPI[CCR] = (1<<31) | (1<<30) | (3<<26) | (3<<24) | (dummy<<18) |
                    (3<<12) | (3<<10) | (3<<8) | (0xEE<<0);
}

void init () {
    RCC(ena::QSPI, 1) = 1;

    Pin::config("B6:PV10,B2:PV9,C9,C10,D13,E2");

    QUADSPI[DCR] = ((fsize-1)<<16); // FSIZE
    QUADSPI[LPTR] = (1<<10); // raise nsel after 1k idle cycles
    QUADSPI[CR] = (2<<24) | (1<<22) | (1<<3) | (1<<0); // PRESCALER APMS TCEN EN

    waitBusy();
    // indirect: IMODE INS
    QUADSPI[CCR] = (1<<8) | (0x35<<0); // enter QPI mode for INS+ADDR+DATA

    mmapEnable();
}

void deinit () {
    waitBusy();
    QUADSPI[CCR] = (3<<8) | (0xF5<<0); // reset QPI mode
    waitBusy();
#if 0 // FIXME doesn't work on next init, something else needs to be reset
    QUADSPI[CCR] = (3<<8) | (0xF5<<0); // reset QPI mode
    waitBusy();
    QUADSPI[CCR] = (3<<8) | (0x66<<0); // enable reset
    waitBusy();
    QUADSPI[CCR] = (3<<8) | (0x99<<0); // reset device
    waitBusy();
    RCC[0x18](1) = 1; // AHB3RSTS
    RCC[0x18](1) = 0; // ~AHB3RSTS
//  QUADSPI[CCR] = 0;
//  QUADSPI[CR] = 0;
//  QUADSPI[PIR] = 0;
//  QUADSPI[LPTR] = 0;
//  Pin::config("B6:F,B2,C9,C10,D13,E2"); // float all
#endif
    RCC(ena::QSPI, 1) = 0;
    Pin::config("B6:F"); // NCS
}

// mass erase, this takes 2..3 mins
void wipe () {
    waitBusy();
    // indirect: IMODE INS
    QUADSPI[CCR] = (3<<8) | (0x06<<0); // write enable

    waitBusy();
    QUADSPI[CCR] = (3<<8) | (0xC7<<0); // chip erase

    mmapEnable();
}

void read (uint32_t addr, uint32_t* ptr, int num) {
    waitBusy();
    // indirect: DDRM DHHC FMODE DMODE DCYC, ADSIZE ADMODE IMODE INS
    QUADSPI[CCR] = (1<<31) | (1<<30) | (1<<26) | (3<<24) | (dummy<<18) |
                    (3<<12) | (3<<10) | (3<<8) | (0xEE<<0); // read bytes
    QUADSPI[AR] = addr;
    QUADSPI[DLR] = 4*num-1;

    for (int i = 0; i < num; ++i)
        ptr[i] = QUADSPI[DR];

    mmapEnable();
}

// erase one 4 kB sector
void erase (uint32_t addr) {
    waitBusy();
    // indirect: IMODE INS
    QUADSPI[CCR] = (3<<8) | (0x06<<0); // write enable

    waitBusy();
    // indirect: ADSIZE ADMODE IMODE INS // program bytes
    QUADSPI[CCR] = (3<<12) | (3<<10) | (3<<8) | (0x21<<0); // page erase
    QUADSPI[AR] = addr;

    mmapEnable();
}

// can write at most 64 words (256 bytes)
void write (uint32_t addr, uint32_t const* ptr, int num) {
    waitBusy();
    // indirect: IMODE INS
    QUADSPI[CCR] = (3<<8) | (0x06<<0); // write enable

    waitBusy();
    // indirect: DMODE ADSIZE ADMODE IMODE INS
    QUADSPI[CCR] = (3<<24) | (3<<12) | (3<<10) | (3<<8) | (0x12<<0); // program
    QUADSPI[AR] = addr;
    QUADSPI[DLR] = 4*num-1;

    waitBusy();
    for (int i = 0; i < num; ++i)
        QUADSPI[DR] = ptr[i];

    mmapEnable();
}

} // namespace qspi
