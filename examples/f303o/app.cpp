// Main application code, combines all sorts of time-related functionality.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "ubx.h"
#include "decode.h"

Ticker ticker;
TICKER_INSTALL(ticker)

ExtIrq extier;
EXTIRQ_INSTALL(extier)

struct Rusher : Task {
    enum TAG { START, TRACK, TICK, PPS };

    char ledSel =0;                  // which signal to display on the LED
    uint8_t dcfNow, msfNow;          // values captured during last tick
    Event tracker;                   // task to notify on each edge change
    uint16_t ppsPrev =0;             // cycle count of last PPS pulse
    uint16_t ticks =0, prevTicks =0; // tick count (once every 4 ms, that is)

    Rusher () : Task ("rush") {}

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(4, TICK);
                extier.enable(gpsPps, extier.RISE, PPS);
                assert(gpsPps.pin() == 5);  // TODO hard-coded for "B5"
                irqEnable(Irq::EXTI9_5, 0); // set PPS pin to highest IRQ prio
                break;
            case TRACK:
                tracker = take(out); // side-effect: clear "out"
                break;
            case TICK:
                ++ticks; // keep track of current tick count
                out = ticked(); // may have a reply for tracking task
                break;
            case PPS:
//assert((uint16_t) (cycles::count() - in.eVal) < 10000);
                if (flag("Gp")) {
                    uint16_t lag = cycles::count() - in.eVal;
                    uint16_t diff = (in.eVal - ppsPrev) - SystemCoreClock;
                    logf("G pps lag %d cy, clk diff %d cy", lag, diff);
                }
                ppsPrev = in.eVal;
                break;
            default:
                fail();
        }
        return out;
    }

private:
    Event ticked () {
        auto dcfPrev = dcfNow, msfPrev = msfNow;
        dcfNow = dcfDat;
        msfNow = !msfDat; // inverted signal

        // show signal on LED and report changes if enabled
        switch (ledSel) {
            case 'g':
                led = +gpsPps;
                break;
            case 'd':
                led = dcfNow;
                break;
            case 'm':
                led = msfNow;
                break;
        }

        if (dcfNow == dcfPrev && msfNow == msfPrev)
            return {};

        auto elapsed = (ticks - prevTicks) & 0x3FFF;
        prevTicks = ticks;

        if (flag("Rt"))
            logf("R track d%d m%d elapsed %d", dcfPrev, msfPrev, elapsed);

        tracker.eVal = (dcfPrev<<15) | (msfPrev<<14) | elapsed;
        return tracker;
    }
} rusher;

struct Gpser : Task {
    enum TAG { START, RECV, SETRTC };

    ubx::Parser<100> ubx;
    int32_t lon =0, lat =0;
    uint32_t lastSet =0; // UTC time in seconds
    DateTime now;

    Gpser () : Task ("gps") {}

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                gpsUart.read(0, { wId, RECV });
                break;
            case RECV: {
                auto i = 0U;
                while (i < in.eVal)
                    if (ubx.parse(gpsUart.rxPtr[i++])) {
                        if (ubx.pktClass == 0x01 && ubx.pktMsgId == 0x07)
                            getPvtInfo();
                        else if (flag("Gu")) {
                            logf("G ubx %02x %02x",
                                    ubx.pktClass, ubx.pktMsgId);
                            logDump(ubx.payload, ubx.pktLen);
                        }
                        break;
                    }
                gpsUart.read(i, { wId, RECV });
                break;
            }
            case SETRTC:
                if (flag("Gs")) {
                    auto dt1 = rtc::getDate(), dt2 = DateTime{ lastSet };
                    auto t1 = dt1.asText(), t2 = dt2.asText();
                    logf("G set rtc %s gps %s diff %d ms",
                            t1.buf, t2.buf, dt1.todMillis() - dt2.todMillis());
                }
                rtc::set(lastSet);
                break;
            default:
                fail();
        }
        return out;
    }

private:
    void getPvtInfo () {
        auto& pvt = *(ubx::NavPvt*) ubx.payload;
        lon = pvt.flags & 1 ? pvt.lon : 0;
        lat = pvt.flags & 1 ? pvt.lat : 0;

        // now will only be exact when ss != 0 || ms != 0
        now = { pvt.year % 100, pvt.month, pvt.day,
                pvt.hour, pvt.min, pvt.sec };

        // fractional seconds correction, careful with 00s wraparound
        if (pvt.nano < 0) {
            if (now.ss > 0)
                --now.ss;
            else
                now = DateTime{ now-1 }; // yuck
            pvt.nano += 1'000'000'000;
        }
        now.ms = pvt.nano / 1'000'000;

        if ((pvt.valid & 3) != 3) // date & time validity flags
            now.yr = 0; // flag as invalid
        if (flag("Gf")) {
            auto dt = now.asText();
            logf("G fix %d pos %d %d ha %d sv %-2d %s ta %d",
                 pvt.fixType, lat, lon, pvt.hAcc, pvt.numSV, dt.buf, pvt.tAcc);
        }

        // set once an hour, but only when GPS has accurate info
        if (now >= lastSet + 300-1 && pvt.tAcc < 100 &&
                            now.yr != 0 && (lon|lat) != 0) {
            lastSet = now + 1;
            ticker.delay(1000 - now.ms, SETRTC); // always an exact second
        }
    }

} gpser;

struct Adjuster : Task {
    enum TAG { START, ADJUST };

    int8_t lsePrev =0;

    Adjuster () : Task ("adjust") {}

    uint8_t init () {
        RCC[0x04](24,3) = 3; // CFGR MCO=LSE

        // use F303RC's TIM3 in ext clock mode 1, count 1 PPS up to 32
        RCC(ena::TIM3,1) = 1;
        TIM3[SMCR] = (6<<4) | (7<<0); // TS=TI2FP2 SMS=ExtClk1
        TIM3[ARR] = 31;      // auto-reload
        TIM3[CR2] = (2<<4);  // MMS update
        TIM3[DIER] = (1<<0); // UIE
        TIM3[CR1] = 1;       // CEN

        // use TIM2 as counter for the lseIn pin, as slave reset by TIM3
        RCC(ena::TIM2,1) = 1;
        TIM2[SMCR] = (1<<16) | (1<<14) | (2<<4); // SMS[3] ECE TS=TIM3
        TIM2[CCMR1] = (3<<8); // CC2S = TCR
        TIM2[CCER] = (1<<4);  // CC2E
        TIM2[CR1] = 1;        // CEN

        irqEnable(Irq::TIM3);
        return Task::init();
    }

    int lseDiff () const {
        return TIM2[CCR2] - (1<<20); // difference from 32x 32 kHz counts
    }

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                break;
            case ADJUST: {
                auto diff = lseDiff();
                if (diff != lsePrev && -100 < diff && diff < 100) {
                    lsePrev = diff;
                    rtc::calibrate(diff);
                }
                if (flag("Ga")) {
                    auto t1 = rtc::getDate().asText(), t2 = gpser.now.asText();
                    logf("G adjust %d rtc %s gps %s", diff, t1.buf, t2.buf);
                }
                break;
            }
            default:
                fail();
        }
        return out;
    }

    void irqCapture () {
        TIM3[SR] = 0; // clear interrupt
        trigger(ADJUST);
    }

    enum { CR1=0x00, CR2=0x04, SMCR=0x08, DIER=0x0C, SR=0x10,
           CCMR1=0x18, CCER=0x20, CNT=0x24, ARR=0x2C, CCR2=0x38 };
} adjuster;
IRQ_HANDLER(TIM3, adjuster.irqCapture)

struct Blinker : Task {
    enum TAG { START, TICK };

    bool enable =false;

    Blinker () : Task ("blink") {}

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(250, TICK);
                break;
            case TICK:
                if (enable)
                    led.toggle();
                break;
            default:
                fail();
        }
        return out;
    }
} blinker;

struct Cmder : Task {
    enum TAG { START, TTYIN, REPORT };

    uint32_t value =0, lastVal =0;
    char lastCh =0;

    Cmder () : Task ("cmd") {}

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ttyUart.read(0, { wId, TTYIN });
                break;
            case TTYIN:
                doCmd(*ttyUart.rxPtr);
                ttyUart.read(1, { wId, TTYIN });
                break;
            case REPORT:
                logf("lse %2d: cnt %08x diff %d",
                        +TIM3[adjuster.CNT],
                        +TIM2[adjuster.CNT],
                        adjuster.lseDiff());
                break;
            default:
                fail();
        }
        return out;
    }

    void doCmd (char ch) {
        // upper case sets, modifies, or shows flags
        if ('A' <= lastCh && lastCh <= 'Z') {
            if ('a' <= ch && ch <= 'z')
                lastVal |= 1 << (ch-'a');
            else if (ch == '*')
                lastVal |= (1<<26) - 1; // all
            else {
                auto& f = flagsAtoZ[lastCh-'A'];
#if !NOFLAGS
                switch (ch) {
                    case '+': f |= lastVal; break;
                    case '-': f &= ~lastVal; break;
                    case '=': f = lastVal; break;
                }
#endif
                logf("  %c = 0x%08x = %u", lastCh, f, f);
                lastCh = 0;
            }
            return;
        }
        // digits are collected as decimal number
        if ('0' <= ch && ch <= '9') {
            value = 10 * value + (ch - '0');
            return;
        }
        // other commands can still get the value as lastVal
        lastVal = take(value);
        // ignore non-printable characters
        if (ch < ' ' || ch > '~')
            return;

        // dispatch on all other characters as cmd code
        lastCh = ch;
        if ('A' <= ch && ch <= 'Z')
            return; // will act on next incoming char
        switch (ch) {
            case '!':
                systemReset();
            case 'd':
                rusher.ledSel = 'd';
                blinker.enable = false;
                dcfOff.toggle(); // DCF77 enable on/off
                break;
            case 'm':
                rusher.ledSel = 'm';
                blinker.enable = false;
                msfOff.toggle(); // MSF60 enable on/off
                break;
            case 'g': {
                rusher.ledSel = 'g';
                blinker.enable = false;
                gpser.lastSet = 0; // request an RTC date & time set
                if (flag("Gm")) {
                    ubx::Maidenhead mh (gpser.lat, gpser.lon);
                    logf("G maidenhead %s latitude %d, longitude %d",
                            mh.buf, gpser.lat, gpser.lon);
                }
                break;
            }
            case 'l':
                blinker.enable = !blinker.enable;
                if (blinker.enable)
                    rusher.ledSel = 0; // stop tracking other signals
                break;
            case 's':
                showStats();
                break;
            case 'h':
                logf("history: max %d", Task::MAX_HISTORY-1);
                showHistory();
                break;
            case 'r':
                if (lastVal != 0)
                    ticker.periodic(100 * lastVal, REPORT);
                else
                    ticker.cancel(REPORT);
                break;
            case 'f':
                logf("flags: A-Z");
                for (auto c = 'A'; c <= 'Z'; ++c)
                    if (auto f = flagsAtoZ[c-'A']; f != 0)
                        logf("  %c = 0x%08x = %u", c, f, f);
                break;
            default:
                logf("? !=reset d)cf m)sf g)ps l)ed s)tats h)istory r)eport"
                              " f)lags");
        }
    }

} cmder;

struct Watcher : Task {
    enum TAG { START, TICK };

    Watcher () : Task ("watch") {}

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(3000, TICK);
                dog::init(3); // approx 3.28s
                break;
            case TICK:
                dog::kick();
                break;
            default:
                fail();
        }
        return out;
    }
} watcher;

struct Idler : Task {
    enum TAG { START, EDGE };

    Convolution<250,1> dcf;
    Convolution<250,0> msf;

    Idler () : Task ("idle") {}

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                send({ rusher.wId, rusher.TRACK }, { wId, EDGE });
                break;
            case EDGE:
                // triggered each time the DCF or MSF signal changes
                decode((in.eVal>>15) & 1, (in.eVal>>14) & 1, in.eVal & 0x3FFF);
                break;
            default:
                fail();
        }
        return out;
    }

private:
    void decode (bool dcfSig, bool msfSig, uint16_t ticks) {
        if (!dcfOff)
            for (auto i = 0U; i < ticks; ++i)
                if (dcf.feed(dcfSig)) {
                    auto dt = dcf.decode().asText();
                    printf("dcf: %s", dt.buf);
                    for (auto i = 1; i < dcf.BANDS; ++i)
                        printf(" %07x%08x", (uint32_t) (dcf.bits[i]>>32),
                                            (uint32_t) dcf.bits[i]);
                    printf("\n");
                }
        if (!msfOff)
            for (auto i = 0U; i < ticks; ++i)
                if (msf.feed(msfSig)) {
                    auto dt = msf.decode().asText();
                    printf("msf: %s", dt.buf);
                    for (auto i = 1; i < msf.BANDS; ++i)
                        printf(" %07x%08x", (uint32_t) (msf.bits[i]>>32),
                                            (uint32_t) msf.bits[i]);
                    printf("\n");
                }
    }
} idler;

void initGps () {
    gpsUart.init(UART1_PINS, 9600);
    gpsUart.wName = "gps-uart";

    const uint8_t config [] = {
        // enable NAV-PVT
        0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51,
        // UART1: NMEA off, UBX on, switch to 1 Mbaud
        0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08,
        0x00, 0x00, 0x40, 0x42, 0x0f, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x76, 0x4e,
    };
    gpsUart.write(config, sizeof config);
    gpsUart.baudRate(1'000'000);
}

int main () {
    initBoard();

    switch (dog::resetCause()) {
        default: logf("reset cause?"); break;
        case 0:  logf("watchdog"); break;
        case 1:  logf("power up"); break;
        case 2:  logf("system reset"); break;
    }
    //Task::showHistory(); // if called here, no names will be shown ...

    dcfVcc = 1; // power up DCF77 module
    dcfOff = 1; // ... but keep it disabled
    msfVcc = 1; // power up MSF60 module
    msfOff = 1; // ... but keep it disabled

    initGps ();

    // start tasks in decreasing priority
    extier.init();
    ticker.init();
    rusher.init();
    gpser.init();
    adjuster.init();
    blinker.init();
    cmder.init();
    watcher.init();
    idler.init();

    Task::showHistory(); // a bit late, but now all tasks have names
    cmder.doCmd('?'); // shows a help msg

    while (true)
        asm ("wfi");
}
