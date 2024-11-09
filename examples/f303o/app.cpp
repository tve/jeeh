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

struct RushWorker : Worker {
    enum TAG { START, TRACK, TICK, PPS };

    char ledSel =0;         // which signal to display on the LED
    uint8_t dcfNow, msfNow; // values captured during last tick
    Event tracker;          // worker to notify on each edge change
    Pin ppsPin {"A8","D"};  // 1PPS signal from the GPS module
    uint16_t ppsPrev =0;    // cycle count of last PPS pulse
    bool dump =false;

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(1, TICK);
                extier.enable(ppsPin, extier.RISE, PPS);
                break;
            case TRACK:
                tracker = take(out); // side-effect: clear "out"
                break;
            case TICK:
                out = ticked(); // may have a reply for tracking worker
                break;
            case PPS:
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
            case 'g': led = +ppsPin; break;
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

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                gpsUart.read(0, { wId, RECV });
                break;
            case RECV: {
                auto i = 0U;
                while (i < in.eVal)
                    if (ubx.parse(gpsUart.rxPtr[i++])) {
                        if (dump) {
                            if (ubx.pktClass == 0x01 && ubx.pktMsgId == 0x07) {
                                auto& pvt = *(ubx::NavPvt*) ubx.payload;
                                lon = pvt.lon;
                                lat = pvt.lat;
                                now = { pvt.year % 100, pvt.month, pvt.day,
                                        pvt.hour, pvt.min, pvt.sec };
                                auto dt = now.asText();
                                logf("fix %d pos %d %d ha %d sv %d %s ta %d",
                                        pvt.fixType, lon, lat, pvt.hAcc,
                                        pvt.numSV, dt.buf, pvt.tAcc);
                            } else {
                                logf("GPS %02x %02x",
                                        ubx.pktClass, ubx.pktMsgId);
                                logDump(ubx.payload, ubx.pktLen);
                            }
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
} gpser;

struct BlinkWorker : Worker {
    enum TAG { START, TICK };

    bool enable =false;

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
    enum TAG { START, TTYIN };

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                console.read(0, { wId, TTYIN });
                break;
            case TTYIN:
                logf("%d: '%c'", in.eVal, *console.rxPtr);
                switch (*console.rxPtr) {
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
                    default:
                        logf("?");
                }
                console.read(1, { wId, TTYIN });
                break;
            default:
                fail();
        }
        return out;
    }
} cmder;

struct WatchWorker : Worker {
    enum TAG { START, TICK };

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

int main () {
    initBoard();

    switch (dog::resetCause()) {
        default: logf("reset cause?"); break;
        case 0:  logf("watchdog"); break;
        case 1:  logf("power up"); break;
        case 2:  logf("system reset"); break;
    }

    dcfVcc = 1;
    msfVcc = 1;

    gpsUart.init(UART1_PINS, 9600);

#if 1
    const uint8_t gpsConfig [] = { // UART1: 1 Mbaud
        // enable NAV-PVT
        0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51,
        // NMEA off, UBX on, 1 Mbaud
        0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xc0, 0x08,
        0x00, 0x00, 0x40, 0x42, 0x0f, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x76, 0x4e,
    };
    gpsUart.write(gpsConfig, sizeof gpsConfig);
    cycles::msBusy(10);
    gpsUart.baudRate(1'000'000);
#endif

    // start workers in decreasing priority
    ticker.init();  ticker.wName = "tick";
    extier.init();  extier.wName = "exti";
    rusher.init();  rusher.wName = "rush";
    gpser.init();   gpser.wName = "gps";
    blinker.init(); blinker.wName = "blink";
    cmder.init();   cmder.wName = "cmd";
    watcher.init(); watcher.wName = "watch";
    idler.init();   idler.wName = "idle";

    console.wName = "uart-tty";
    gpsUart.wName = "uart-gps";

    Worker::send({ ticker.wId, ticker.RATE, 1 }); // TODO no START?

    Worker::send({ rusher.wId, rusher.START });
    Worker::send({ gpser.wId, gpser.START });
    Worker::send({ blinker.wId, blinker.START });
    Worker::send({ cmder.wId, cmder.START });
    Worker::send({ watcher.wId, watcher.START });
    Worker::send({ idler.wId, idler.START });

    while (true)
        asm ("wfi");
}
