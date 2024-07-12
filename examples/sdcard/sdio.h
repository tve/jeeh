namespace jeeh {

#ifndef STM32F7
#define SDMMC1 SDIO
#endif

struct Sdio : Device, private Chain {
    static constexpr auto CHAN = 4, STREAM = 3;
    static constexpr auto DMA_IRQ = Irq::DMA2_Stream3;
    auto dmaReg (int off) const { return DMA2[off+CHAN_STEP*STREAM]; }

    Sdio () : Device ('D') {}

    void init () {
        //Pin present ("C13");
        //present.mode("U");
        //printf("detect %d\n", +present);

        Pin::config("C8:V12,C9,C10,C11,C12,D2");
        RCC(ena::DMA2, 1) = 1;   // dma on
        RCC(ena::SDMMC1, 1) = 1; // sdio on

#if STM32F7
        // set up 48 MHz as SDMMC clock
        RCC[0x00](28) = 0;            // ~PLLSAION in CR
        RCC[0x88] = (5<<28) | (2<<24) | (3<<16) | (384<<6); // R, Q, P, N
        RCC[0x00](28) = 1;            // PLLSAION in CR
        while (RCC[0x00](29) == 0) {} // wait for PLLSAIRDY in CR
#endif

        SDMMC1[SD_CCR] = (1<<14) | (1<<9) | (118<<0); // HWFC_EN PWRSAV CLKDIV
        SDMMC1[SD_PWR] = 3; // PWRON
        SDMMC1[SD_CCR](8) = 1; // CLKEN

        SDMMC1[SD_DTIM] = 48'000'000;
        SDMMC1[SD_DLEN] = 512;

        sendCmd(0, 0, 0); // GO_IDLE_STATE
        sendCmd(8, 0x1AA, 1); // SEND_IF_COND

        do
{
sys::wait(500);
            sendCmd(55, 0, 1); // APP_CMD
sys::wait(500);
}
        while (sendCmd(41, 1<<30, 1) == 1); // APP_OP_COND
sys::wait(10);

        sendCmd(58, 0, 1); // READ_OCR
        sendCmd(16, 512, 1); // SET_BLOCKLEN

        sendCmd(2, 0, 3); // ALL_SEND_CID
        printf("cid %08x %08x %08x %08x\n", +SDMMC1[SD_RSP], +SDMMC1[SD_RSP+4],
                                        +SDMMC1[SD_RSP+8], +SDMMC1[SD_RSP+12]);
        sendCmd(3, 0, 1); // SET_REL_ADDR
        auto rel = SDMMC1[SD_RSP] & 0xFFFF0000;
        SDMMC1[SD_CCR](0, 8) = 0;  // switch to 24 MHz
        //SDMMC1[SD_CCR](10) = 1;  // switch to 48 MHz

        sendCmd(9, rel, 3); // SEND_CSD
        csd[0] = SDMMC1[SD_RSP];
        csd[1] = SDMMC1[SD_RSP+4];
        csd[2] = SDMMC1[SD_RSP+8];
        csd[3] = SDMMC1[SD_RSP+12];
        printf("csd %08x %08x %08x %08x\n", csd[0], csd[1], csd[2], csd[3]);

        sendCmd(7, rel, 1); // SEL_DESEL_CARD

        sendCmd(55, rel, 1); // APP_CMD
        sendCmd(6, 2, 1); // APP_SET_BUSWIDTH
        SDMMC1[SD_CCR](11, 2) = 1; // switch to 4-bit mode

        dmaReg(FCR) = (1<<2) | (3<<0); // DMDIS FTH
        dmaReg(CPAR) = SDMMC1.ADDR + 0x80; // FIFO
        dmaReg(CCR) = (CHAN<<25) | // CHSEL
                      (1<<21) | (2<<13) | (2<<11) | // PBURST MSIZE PSIZE 
                      (1<<10) | (1<<5) | (1<<4); //  MINC PFCTRL TCIE

        irqInstall((int) Irq::SDMMC1);
        irqInstall((int) DMA_IRQ);
    }

    void deinit () {
        RCC(ena::SDMMC1, 1) = 0; // sdio off
        //RCC(ena::DMA2, 1) = 0;   // dma off
    }

    void start (Message& m) override {
        switch (m.mTag) {
            case 'I': m.mPtr = (uint8_t*) capacity(); reply(&m); break;
            case 'B':
                seek = (uint32_t) m.mPtr;
                m.mLen = SDMMC1[SD_STA](12);
                reply(&m);
                break;
            case 'R':
            case 'W':
                if (!append(m))
                    ioStart();
                break;
            default:  m.mTag = -1; reply(&m); break;
        }
    }

    void finish () override {
        reply(pull());
        ioStart();
    }

    void cancel (Message& m) override {
        // FIXME not quite right for both head and non-head removal
        if (remove(m))
            ioStart();
    }

private:
    uint32_t csd [4] {~0U};
    uint32_t seek;

    uint32_t sendCmd (int cmd, int arg, int res) {
        constexpr auto CMD_STATUS = (1<<7) | (1<<6) | (1<<2) | (1<<0);
        SDMMC1[SD_ARG] = arg;
        SDMMC1[SD_CMD] = // CPSMEN WAITRESP CMDINDEX
            (1<<10) | (res<<6) | (cmd<<0);
        auto t = cycles::count();
        while ((SDMMC1[SD_STA] & CMD_STATUS) == 0) {}
        t = cycles::count() - t;
        uint32_t s = SDMMC1[SD_STA];
        printf("  cmd %2d: s %08x, %d us, r %08x\n",
                cmd, s, t/200, +SDMMC1[SD_RSP]);
        SDMMC1[SD_ICR] = CMD_STATUS; // clear flags
        return s;
    }

    uint32_t capacity () const {
        uint32_t s, m;
        switch (csd[0] >> 30) {
            case 0:  s = ((csd[1] & 0x03FF) << 2) | (csd[2] >> 30);
                     m = ((csd[2] & 0x0003'8000) >> 15) + 2;
                     break;
            case 1:  s = ((csd[1] & 0x003F) << 16) | (csd[2] >> 16);
                     m = 10;
                     break;
            default: return 0;
        }
        return (s+1) << m;
    }

    void ioStart () {
        auto mp = first();
        if (mp != nullptr) {
            assert(!SDMMC1[SD_STA](12)); // can't be busy

            auto read = mp->mTag == 'R';
            if (read)
                cache::inval(mp->mPtr, mp->mLen);
            else
                cache::flush(mp->mPtr, mp->mLen);

            dmaReg(CMAR) = (uint32_t) mp->mPtr;
            dmaReg(CNDTR) = mp->mLen / 4;
            dmaReg(CCR)(6, 2) = !read; // DIR
            dmaReg(CCR)(0) = 1; // EN

            // similar to sendCmd, but with DMA_IRQ when response received
            SDMMC1[SD_MASK](8) = 1; // DATAENDIE
            SDMMC1[SD_ARG] = (csd[0] >> 30 ? 1 : 512) * seek;
            SDMMC1[SD_CMD] = // CPSMEN WAITRESP CMDIDX
                (1<<10) | (1<<6) | ((read ? 17 : 24)<<0);
            SDMMC1[SD_DCTR] = // DBLOCKSIZE DMAEN DTDIR
                (9<<4) | (1<<3) | (read<<1);
        }
    }

    // the actual interrupt handler, with access to the sdio object
    bool interrupt (int irq) override {
        constexpr uint8_t ifcBits [] = { 0, 6, 16, 22 };
        switch (irq) {
            case (int) Irq::SDMMC1:
                SDMMC1[SD_ICR] = (1<<7) | (1<<6) | (1<<2) | (1<<0);
                SDMMC1[SD_DCTR](0) = 1; // DTEN
                return true; // sdio done, dma started
            case (int) DMA_IRQ:
                DMA2[IFCR+(STREAM&~3)] = 0b111101 << ifcBits[STREAM&3];
                return false; // dma done, but card still in write cmd
        }
        return false;
    }

    enum { IFCR=0x08,CCR=0x10,CNDTR=0x14,CPAR=0x18,CMAR=0x1C,FCR=0x24 };
    enum { CHAN_STEP=0x18 };
    enum { SD_PWR=0x00, SD_CCR=0x04, SD_ARG=0x08, SD_CMD=0x0C, SD_RSP=0x14,
           SD_DTIM=0x24, SD_DLEN=0x28, SD_DCTR=0x2C, SD_DCNT=0x30,
           SD_STA=0x34, SD_ICR=0x38, SD_MASK=0x3C, SD_FIFO=0x80 };
};

} // namespace jeeh
