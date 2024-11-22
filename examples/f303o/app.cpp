// Main application code, combines all sorts of time-related functionality.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "ubx.h"
#include "decode.h"

extern "C" uint8_t* _sbrk (uint32_t);

ExtIrq extier;
EXTIRQ_INSTALL(extier)

Ticker ticker;
TICKER_INSTALL(ticker)

struct Rusher : Task {
    enum TAG { START, TRACK, TICK, PPS };

    char ledSel =0;                  // which signal to display on the LED
    uint8_t dcfNow, msfNow;          // values captured during last tick
    Event tracker;                   // task to notify on each edge change
    uint32_t ppsMillis =0;           // ms of last 1PPS pulse
    uint16_t ppsPrev =0;             // cycle count of last PPS pulse
    int16_t ppsDiff =0;              // cycle diff from one pulse to the next
    uint16_t ticks =0, prevTicks =0; // tick count (once every 4 ms)
    int16_t hsePpm =0;               // calculated HSE clock error

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
                ppsMillis = cycles::millis();
                if (flag("Gp")) {
                    uint16_t lag = cycles::count() - in.eVal;
                    uint16_t diff = (in.eVal - ppsPrev) - SystemCoreClock;
                    logf("G pps lag %d cy, clk diff %d cy", lag, diff);
                }
                ppsDiff = in.eVal - ppsPrev;
                ppsPrev = in.eVal;
                hsePpm = (int16_t) (ppsDiff - SystemCoreClock) /
                            (int) (SystemCoreClock/1'000'000);
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
                led2 = +gpsPps;
                break;
            case 'd':
                led2 = dcfNow;
                break;
            case 'm':
                led2 = msfNow;
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
    uint32_t lastSet =0, hAcc =0, tAcc =0, sv =0, fix =0;
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
        hAcc = pvt.flags & 1 ? pvt.hAcc : 0;
        tAcc = (pvt.valid & 3) == 3 ? pvt.tAcc : 0;
        sv = pvt.numSV;
        fix = pvt.fixType;

        // now will only be exact when ss != 0 || ms != 0
        now = { pvt.year % 100, pvt.month, pvt.day,
                pvt.hour, pvt.min, pvt.sec };

        // set once every hour, but only when GPS has accurate info
        auto ppsLag = cycles::millis() - rusher.ppsMillis + pvt.nano/1'000'000;
        if (now >= lastSet + 3600-1 && 20 < ppsLag && ppsLag < 60 &&
                        pvt.tAcc < 1000 && now.yr != 0 && (lon|lat) != 0) {
            lastSet = now + 1;
            ticker.delay(1000 - ppsLag, SETRTC); // on exact second
        }

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
            logf("G fix %d lat %d ha %d sv %-2d %s pps %d ta %d",
                pvt.fixType, lat, pvt.hAcc, pvt.numSV, dt.buf, ppsLag, pvt.tAcc);
        }
    }

} gpser;

struct Adjuster : Task {
    enum TAG { START, ADJUST };

    int16_t lsePpm =0;

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

    int lseDiff () const { // difference from 32x 32 kHz counts
        //return ((TIM2[CCR2] - (1<<20)) * 512) / 489;
        return TIM2[CCR2] - (1<<20);
    }

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                break;
            case ADJUST: {
                auto diff = lseDiff();
                if (diff != lsePpm && -100 < diff && diff < 100) {
                    lsePpm = diff;
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
                    led2.toggle();
                break;
            default:
                fail();
        }
        return out;
    }
} blinker;

struct Cmder : Task {
    enum TAG { START, TTYIN, REPORT };

    uint32_t value =0, lastVal =0, seqNum =0;
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
            case REPORT: {
                auto t1 = gpser.now.asText();
                auto t2 = rtc::getDate().asText();
                auto t3 = DateTime{ gpser.lastSet }.asText();
                ubx::Maidenhead mh (gpser.lat, gpser.lon);
                logf("#%d %s", ++seqNum, led1 ? "OK" : "");
                constexpr char const* fixDesc [] = {
                    "NO", "DRO", "2D", "3D", "GNSS", "TIME"
                };
                logf("lse %d ppm, hse %d ppm, hor %d m, tim %d ns, sat %d : %s",
                        adjuster.lsePpm, rusher.hsePpm,
                        gpser.hAcc/1000, gpser.tAcc, gpser.sv,
                        fixDesc[gpser.fix]);
                logf("gps %s  lon %d  lat %d  mh %s",
                        t1.buf, gpser.lon, gpser.lat, led1 ? mh.buf : "-");
                logf("rtc %s  set %s", t2.buf, t3.buf);
                memInfo();
                break;
            }
            default:
                fail();
        }
        return out;
    }

    void memInfo () const {
        auto heapEnd = _sbrk(0);
        auto currSp = (uint8_t*) &heapEnd;
        extern uint8_t g_pfnVectors [], _siccmram [], _sdata [], _sbss [],
                       _ebss [], _estack [];
        logf("code %d  data %d  bss %d  heap %d  stack %d  free %d kb",
                _siccmram-g_pfnVectors, _sbss - _sdata, _ebss - _sbss,
                heapEnd - _ebss, _estack - currSp, (currSp-heapEnd) >> 10);
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
                ticker.cancel(REPORT);
                if (lastVal > 0)
                    ticker.periodic(100 * lastVal, REPORT);
                ticker.delay(1, REPORT);
                break;
            case 'f':
                logf("flags: A-Z");
                for (auto c = 'A'; c <= 'Z'; ++c)
                    if (auto f = flagsAtoZ[c-'A']; f != 0)
                        logf("  %c = 0x%08x = %u", c, f, f);
                break;
            case 't': {
                ubx::Packet<ubx::CfgNav5> pkt;
                pkt.data.mask = 0b1; // dyn
                pkt.data.dynModel = 2; // stationary
                auto [p, n] = pkt.wrapper();
                logf("11 %p %d", p, n);
                gpsUart.write(p, n);
                break;
            }
            default:
                logf("? !=reset d)cf m)sf g)ps l)ed s)tats h)istory r)eport"
                              " f)lags t)est");
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
            case TICK: {
                auto lse = adjuster.lsePpm, hse = rusher.hsePpm;
                auto hacc = gpser.hAcc/1000, tacc = gpser.tAcc;
                auto lat = gpser.lat, lon = gpser.lon;
                // LED is on when all readings are in their acceptable range
                led1 = (lse * hse * hacc * tacc) != 0 && (lat|lon) != 0 &&
                       (-50 < lse && lse < 50) && (-50 < hse && hse < 50) &&
                       hacc < 100 && tacc < 1000;
                dog::kick();
                break;
            }
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
    logf("tasks: E:%d T:%d R:%d G:%d A:%d B:%d C:%d W:%d I:%d", sizeof extier,
            sizeof ticker,  sizeof rusher, sizeof gpser, sizeof adjuster,
            sizeof blinker, sizeof cmder, sizeof watcher, sizeof idler);
    cmder.doCmd('?'); // shows a help msg

    while (true)
        asm ("wfi");
}
