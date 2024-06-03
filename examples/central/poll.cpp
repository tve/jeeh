// Polled access to the RFM69 radio, via bit-banged or hardware SPI.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#define RF69_SPI_BULK 1
#include "spi-rf69-v1.h"

// TODO this is hard-coded for SPI2
struct SpiSync : SpiGpio {
    struct Config {
        uint32_t addr;
        uint16_t ena;
        uint8_t mhz;
        Irq txIrq, rxIrq;
        uint8_t dma :1, txChan :3, rxChan :3, txReq, rxReq; // 0-based
    } const dev;

    SpiSync (Config const& config) : dev (config) {}

    enum { CR1=0x00, CR2=0x04, SR=0x08, DR=0x0C }; // SPI regs
    auto devReg (int off) const { IoReg<0> io; return io[dev.addr+off]; }

    enum { IFCR=0x08,CCR=0x10,CNDTR=0x14,CPAR=0x18,CMAR=0x1C }; // DMA regs
    enum { STREAM_STEP=0x18 };

    auto dmaReg (int off) const { return DMA1[0x400*dev.dma+off]; }
    auto dmaTX (int off) const { return dmaReg(off+STREAM_STEP*dev.txChan); }
    auto dmaRX (int off) const { return dmaReg(off+STREAM_STEP*dev.rxChan); }

    void init (char const* pins, int speed) {
        SpiGpio::init(pins);
        Pin::config(pins);

        auto div = 0; // determine clock divider
        while ((dev.mhz >> (div+1)) > speed)
            ++div;

        RCC(dev.ena, 1) = 1;
        devReg(CR1) = (div<<3) | (1<<2); // BD MSTR
        devReg(CR2) = (1<<12) | (7<<8) | (1<<2); // FRXTH DS SSOE
        devReg(CR1)(6) = 1; // SPE

        devReg(CR2)(0) = 1; // RXDMAEN
        devReg(CR2)(1) = 1; // TXDMAEN

        RCC(ena::DMA1+dev.dma, 1) = 1;
        dmaTX(CPAR) = dev.addr + DR;
        dmaTX(CCR) = (dev.txReq<<25) | 0b0100'0101'0000; // CHSEL MINC DIR TCIE
        dmaRX(CPAR) = dev.addr + DR;
        dmaRX(CCR) = (dev.rxReq<<25) | 0b0100'0001'0000; // CHSEL MINC TCIE

        SCB[0x10](4) = 1; // SEVONPEND
    }

    int transfer (int v) const {
        *(volatile uint8_t*) (dev.addr+DR) = v;
        while (devReg(SR)(0) == 0) {} // RXNE
        return *(volatile uint8_t*) (dev.addr+DR);
    }

    void transfer (uint8_t const* out, uint8_t* in, int len) const {
        assert(out != nullptr || in != nullptr);
if (out == nullptr) out = in; // TODO hack, don't know how to do RXONLY w/ DMA
        static uint8_t const ifcBits [] = { 0, 6, 16, 22 };

        if (in != nullptr) {
            dmaRX(CMAR) = (uint32_t) in;
            dmaRX(CNDTR) = len;
            dmaRX(CCR)(0) = 1; // EN
        }
        if (out != nullptr) {
            dmaTX(CMAR) = (uint32_t) out;
            dmaTX(CNDTR) = len;
            dmaTX(CCR)(0) = 1; // EN

            do
                asm ("wfe");
            while (dmaTX(CCR)(0)); // EN

            auto t = dev.txChan;
            dmaReg(IFCR+(t&~3)) = 0b111101 << ifcBits[t&3]; // clr irq
            auto n = (uint8_t) dev.txIrq;
            NVIC[0x180 + 4*(n/32)] = 1 << n%32; // clear pending
        }
        if (in != nullptr) {
            do
                asm ("wfe");
            while (dmaRX(CCR)(0)); // EN

            auto r = dev.rxChan;
            dmaReg(IFCR+(r&~3)) = 0b111101 << ifcBits[r&3]; // clr irq
            auto n = (uint8_t) dev.rxIrq;
            NVIC[0x180 + 4*(n/32)] = 1 << n%32; // clear pending
        } else { // clear OVR flag, as the data was never read
            (void) +devReg(DR);
            (void) +devReg(SR);
        }
    }
};

//SpiGpio spi;
SpiSync spi ({ SPI_NAME.ADDR, ena::SPI_NAME, SPI_FREQ, SPI_CONF });
RF69 rf (spi);

constexpr Pin nrst ("F11");
// not attached:
//constexpr Pin dio0 ("A4");
//constexpr Pin dio1 ("B0");
//constexpr Pin dio2 ("B11");
//constexpr Pin dio3 ("H4");
//constexpr Pin dio5 ("H5");

int main () {
    initBoard("poll"); // in defs.h

    nrst.mode("P");
    //dio0.mode("D");
    //dio1.mode("D");
    //dio2.mode("D");
    //dio3.mode("D");
    //dio5.mode("D");

    nrst = 1;
    sys::wait(10);
    nrst = 0;
    sys::wait(10);

    //spi.init("C3,C2,I1,I0"); // div=0 @ 16 MHz: 8 Mhz
    spi.init(SPI_PINS, 10);
    rf.init(63, 42, 8686);  // node 63, group 42, 868.3 MHz
    rf.txPower(0);

    while (true) {
        uint8_t buf [60];
        auto n = rf.receive(buf, sizeof buf);
        if (n > 0) {
            logf("rssi %d lna %d afc %d @ %d",
                    rf.rssi, rf.lna, rf.afc, rtc::getSecs());
            logDump(buf, n);
        }
        sys::wait(100);
    }
}
