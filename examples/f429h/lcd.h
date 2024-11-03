namespace lcd {

void initPins () {
    RCC(ena::FMC,1) = 1;
    Pin::config("F10:V14,G6,G7,G11,"
                "H9,H10,H11,H12,H13,H14,H15,"
                "I0,I1,I2,I4,I5,I6,I7,I9,I10");
}

enum { WIDTH=800,HEIGHT=480,HSYN=48,HBP=88,HFP=40,VSYN=3,VBP=32,VFP=13 };

// shorthand to set both horizontal and vertical parameters
constexpr auto hv (uint16_t h, uint16_t l) { return (h << 16) | l; }

void init () {
    initPins();

    RCC(ena::LTDC,1) = 1;
    RCC[0x88] = (3<<28) | (7<<24) | (192<<6); // PLLSAICFGR
    RCC[0x8C] = 0;

    RCC[0x00](28) = 1;            // PLLSAION in CR
    while (RCC[0x00](29) == 0) {} // wait for PLLSAIRDY in CR

    LTDC[0x08] = hv(HSYN-1, VSYN-1);                              // SSCR
    LTDC[0x0C] = hv(HSYN+HBP-1, VSYN+VBP-1);                      // BPCR
    LTDC[0x10] = hv(HSYN+HBP+WIDTH-1, VSYN+VBP+HEIGHT-1);         // AWCR
    LTDC[0x14] = hv(HSYN+HBP+WIDTH+HFP-1, VSYN+VBP+HEIGHT+VFP-1); // TWCR

    LTDC[0x18] = (1<<16) | (1<<0); // DEN & LTDCEN in GCR
    LTDC[0x24](0) = 1;             // IMR in SRCR
}

template <int N>
struct FrameBuffer {
    constexpr static IoReg<LTDC.ADDR+0x80*N> LAYER {};
    static_assert(N == 1 || N == 2);

    void init () {
        LAYER[0x04] = 0;                               // ~LEN in LxCR
        LAYER[0x08] = hv(HSYN+HBP+WIDTH-1, HSYN+HBP);  // LxWHPCR
        LAYER[0x0C] = hv(VSYN+VBP+HEIGHT-1, VSYN+VBP); // LxWVPCR
        LAYER[0x10] = 0;                               // LxCKCR
        LAYER[0x14] = N == 1 ? 0b101 : 0b110;          // LxPFCR L8/AL44
        LAYER[0x2C] = (uint32_t) data;                 // LxCFBAR
        LAYER[0x30] = hv(WIDTH, WIDTH+3);              // LxCFBLR
        LAYER[0x34] = HEIGHT;                          // LxCFBLNR

        if constexpr (N == 1)
            for (int i = 0; i < 256; ++i) {
                auto r = i>>5, g = (i>>2) & 7, b = i & 3;
                auto rgb = (r<<21)|(r<<18)|(g<<13)|(g<<10)|(b<<6)|(b<<4)|(b<<2);
                // rgb = (i << 16) | (i << 8) | (i << 0); // greyscale
                LAYER[0x44] = (i<<24) | rgb;
            }
        else
            for (int i = 0; i < 16; ++i) {
                uint8_t r = -((i>>2)&1), g = -((i>>1)&1), b = -(i&1);
                if (i < 8) {
                    r >>= 2; g >>= 2; b >>= 2;
                }
                LAYER[0x44] = (i<<24) | (r<<16) | (g<<8) | b;
            }

        LAYER[0x04] = (1<<4) | (1<<0); // CLUTEN & LEN in LxCR
        LAYER[0x1C] = 0;               // LxDCCR transparent

        LTDC[0x24](0) = 1; // IMR in SRCR
    }

    auto& operator() (int x, int y) { return data[y][x]; }

    uint8_t data [HEIGHT][WIDTH];
};

} // namespace lcd
