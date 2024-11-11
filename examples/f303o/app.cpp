// Main application code, combines all sorts of time-related functionality.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "ubx.h"

Ticker ticker;
TICKER_INSTALL(ticker)

ExtIrq extier;
EXTIRQ_INSTALL(extier)

uint32_t flags [26]; // A..Z: settings for global use

struct RushWorker : Worker {
    enum TAG { START, TRACK, TICK, PPS };

    char ledSel =0;         // which signal to display on the LED
    uint8_t dcfNow, msfNow; // values captured during last tick
    Event tracker;          // worker to notify on each edge change
    uint16_t ppsPrev =0;    // cycle count of last PPS pulse
    bool dump =false;

    RushWorker () : Worker ("rush") {}

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(1, TICK);
                extier.enable(gpsPps, extier.RISE, PPS);
                assert(gpsPps.pin() == 8);  // TODO hard-coded for "A8"
                irqEnable(Irq::EXTI9_5, 0); // set PPS pin to highest IRQ prio
                break;
            case TRACK:
                tracker = take(out); // side-effect: clear "out"
                break;
            case TICK:
                out = ticked(); // may have a reply for tracking worker
                break;
            case PPS:
assert((uint16_t) (cycles::count() - in.eVal) < 10000);
                if (dump) {
                    uint16_t lag = cycles::count() - in.eVal;
                    uint16_t diff = (in.eVal - ppsPrev) - SystemCoreClock;
                    logf("pps lag %d cy, clk diff %d cy", lag, diff);
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
        ++tracker.eVal; // keep track of current tick count

        auto dcfPrev = dcfNow, msfPrev = msfNow;
        dcfNow = dcfDat;
        msfNow = !msfDat; // inverted signal
        if (dcfNow == dcfPrev && msfNow == msfPrev)
            return {};

        // show signal on LED if enabled
        switch (ledSel) {
            case 'd': led = dcfNow; break;
            case 'm': led = msfNow; break;
            case 'g': led = +gpsPps; break;
        }

        return tracker;
    }
} rusher;

struct GpsWorker : Worker {
    enum TAG { START, RECV };

    ubx::Parser<100> ubx;
    int32_t lon =0, lat =0;
    DateTime now;
    bool dump =false;

    GpsWorker () : Worker ("gps") {}

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
                        else if (dump) {
                            logf("GPS %02x %02x",
                                    ubx.pktClass, ubx.pktMsgId);
                            logDump(ubx.payload, ubx.pktLen);
                        }
                        break;
                    }
                gpsUart.read(i, { wId, RECV });
                break;
            }
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

        // now will only be exact when ss != || ff != 0
        now = { pvt.year % 100, pvt.month, pvt.day,
                pvt.hour, pvt.min, pvt.sec };
        if (pvt.nano < 0 && now.ss > 0) {
            --now.ss;
            pvt.nano += 1'000'000'000;
        }
        if (pvt.nano > 0)
            now.ff = pvt.nano / (1'000'000'000 / 256);
        if ((pvt.valid & 3) != 3)
            now.yr = 0; // flag as invalid
        if (dump) {
            auto dt = now.asText();
            logf("fix %d pos %d %d ha %d sv %d %s ta %d",
                    pvt.fixType, lat, lon, pvt.hAcc,
                    pvt.numSV, dt.buf, pvt.tAcc);
        }
    }

} gpser;

struct BlinkWorker : Worker {
    enum TAG { START, TICK };

    bool enable =false;

    BlinkWorker () : Worker ("blink") {}

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

struct CmdWorker : Worker {
    enum TAG { START, TTYIN, REPORT };

    uint32_t value =0, lastVal =0;
    char lastCh =0;

    CmdWorker () : Worker ("cmd") {}

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
                logf("bingo");
                break;
            default:
                fail();
        }
        return out;
    }

    void doCmd (char ch) {
        auto modifier = lastCh == '+' || lastCh == '-' || lastCh == '=';
        if (modifier && 'A' <= ch && ch <= 'Z') {
            switch (take(lastCh)) {
                case '+': flags[ch-'A'] |= 1 << lastVal; break;
                case '-': flags[ch-'A'] &= ~(1 << lastVal); break;
            }
            logf("  %c = 0x%08x = %u", ch, flags[ch-'A'], flags[ch-'A']);
            return;
        }
        // digits are collected as decimal number
        if ('0' <= ch && ch <= '9') {
            value = 10 * value + (ch - '0');
            return;
        }
        // upper case stores the current value in register A..Z
        if ('A' <= ch && ch <= 'Z') {
            flags[ch-'A'] = value;
            value = 0;
            return;
        }
        // other commands can still get the value as lastVal
        lastVal = value;
        value = 0;
        // ignore non-printable characters
        if (ch < ' ' || ch > '~')
            return;

        // dispatch on all other characters as cmd code
        lastCh = ch;
        switch (ch) {
            case '+':
            case '-':
            case '=':
                break; // will act on next incoming char
            case '!':
                systemReset();
            case 'd':
                rusher.ledSel = 'd';
                blinker.enable = false;
                break;
            case 'm':
                rusher.ledSel = 'm';
                blinker.enable = false;
                break;
            case 'g':
                rusher.ledSel = 'g';
                gpser.dump = !gpser.dump;
                if (!gpser.dump) {
                    rusher.dump = !rusher.dump;
                    ubx::Maidenhead mh (gpser.lat, gpser.lon);
                    logf("latitude %d, longitude %d, maidenhead %s",
                            gpser.lat, gpser.lon, mh.buf);
                }
                break;
            case 'l':
                blinker.enable = !blinker.enable;
                if (blinker.enable)
                    rusher.ledSel = 0; // stop tracking other signals
                break;
            case 's':
                showStats();
                break;
            case 'h':
                showHistory();
                break;
            case 'r':
                if (lastVal != 0)
                    ticker.periodic(100 * lastVal, REPORT);
                else
                    ticker.cancel(REPORT);
                break;
            case 'f':
                for (auto c = 'A'; c <= 'Z'; ++c)
                    if (flags[c-'A'] != 0)
                        logf("  %c = 0x%08x = %u", c, flags[c-'A'], flags[c-'A']);
                break;
            default:
                logf("? !=reset d)cf m)sf g)ps l)ed s)tats h)istory r)eport"
                              " f)lags");
        }
    }

} cmder;

struct WatchWorker : Worker {
    enum TAG { START, TICK };

    WatchWorker () : Worker ("watch") {}

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

struct IdleWorker : Worker {
    enum TAG { START, EDGE };

    IdleWorker () : Worker ("idle") {}

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                send({ rusher.wId, rusher.TRACK }, { wId, EDGE });
                break;
            case EDGE:
                // TODO triggered each time the DCF or MSF signal changes
                logf("E %d", in.eVal);
                break;
            default:
                fail();
        }
        return out;
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
    //Worker::showHistory(); // if called here, no names will be shown ...


    dcfVcc = 1; // enable DCF77 module
    msfVcc = 1; // enable MSF60 module

    initGps ();

    // start workers in decreasing priority
    extier.init();
    ticker.init();
    rusher.init();
    gpser.init();
    blinker.init();
    cmder.init();
    watcher.init();
    idler.init();

    Worker::showHistory(); // a bit late, but now all workers have names
    cmder.doCmd('?'); // shows a help msg

    while (true)
        asm ("wfi");
}
