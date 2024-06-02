// Polled access to the RFM69 radio, via bit-banged or hardware SPI.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

#define RF69_SPI_BULK 1
#include "spi-rf69-v1.h"

struct SpiHw : SpiGpio {
    enum { CR1=0x00, CR2=0x04, SR=0x08, DR=0x0C }; // SPI regs

    void init (char const* desc) {
        SpiGpio::init(desc);
        Pin::config(":5,,", &mosi, 3);

        RCC(ena::SPI2, 1) = 1;
        SPI2[CR1] = (2<<3) | (1<<2); // BD/8 (APB1 = 54 MHz) MSTR
        SPI2[CR2] = (1<<12) | (7<<8) | (1<<2); // FRXTH DS SSOE
        SPI2[CR1](6) = 1; // SPE
    }

    int transfer (int v) {
        SPI2.byte(DR) = v;
        while (SPI2[SR](0) == 0) {} // RXNE
        return SPI2.byte(DR);
    }

    void transfer (uint8_t const* out, uint8_t* in, int len) {
        for (auto i = 0; i < len; ++i) {
            auto b = transfer(out != nullptr ? out[i] : 0);
            if (in != nullptr)
                in[i] = b;
        }
    }
};

// TODO this is hard-coded for SPI2 and DMA1
struct SpiDma : SpiHw {
    enum { IFCR=0x08,CCR=0x10,CNDTR=0x14,CPAR=0x18,CMAR=0x1C }; // DMA regs
    enum { STREAM_STEP=0x18 };
    enum { TX_STR=4, RX_STR=3, TX_CH=0, RX_CH=0 }; // hard-coded SPI2 config

    auto dmaTX (int off) const { return DMA1[off+STREAM_STEP*TX_STR]; }
    auto dmaRX (int off) const { return DMA1[off+STREAM_STEP*RX_STR]; }

    void init (char const* desc) {
        SpiHw::init(desc);

        SPI2[CR2](0) = 1; // RXDMAEN
        SPI2[CR2](1) = 1; // TXDMAEN

        RCC(ena::DMA1, 1) = 1;
        dmaTX(CPAR) = SPI2.ADDR + DR;
        dmaTX(CCR) = (TX_CH<<25) | 0b0100'0101'0000; // CHSEL MINC DIR TCIE
        dmaRX(CPAR) = SPI2.ADDR + DR;
        dmaRX(CCR) = (RX_CH<<25) | 0b0100'0001'0000; // CHSEL MINC TCIE

        SCB[0x10](4) = 1; // SEVONPEND
    }

    using SpiHw::transfer; // use the polled version for single-byte transfers

    void transfer (uint8_t const* out, uint8_t* in, int len) {
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

            while (dmaTX(CCR)(0)) // EN
                asm ("wfe");
            DMA1[IFCR+(TX_STR&~3)] = 0b111101 << ifcBits[TX_STR&3]; // clr irq
            auto n = (uint8_t) Irq::DMA1_Stream4;
            NVIC[0x180 + 4*(n/32)] = 1 << n%32;
        }
        if (in != nullptr) {
            while (dmaRX(CCR)(0)) // EN
                asm ("wfe");
            DMA1[IFCR+(RX_STR&~3)] = 0b111101 << ifcBits[RX_STR&3]; // clr irq
            auto n = (uint8_t) Irq::DMA1_Stream3;
            NVIC[0x180 + 4*(n/32)] = 1 << n%32;
        } else { // clear OVR flag, as the data was never read
            (void) +SPI2[DR];
            (void) +SPI2[SR];
        }
    }
};

//SpiGpio spi;
//SpiHw spi;
SpiDma spi;
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

    spi.init("C3,C2,I1,I0"); // div=0 @ 16 MHz: 8 Mhz
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
