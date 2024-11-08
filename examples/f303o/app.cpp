// Main application code, combines all sorts of time-related functionality.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"
#include "ubx.h"

Ticker ticker;
TICKER_INSTALL(ticker)

struct RushWorker : Worker {
    enum TAG { START, TRACK, TICK };

    char ledSel =0;         // which signal to display on the LED
    uint8_t dcfNow, msfNow; // values captured during last tick
    Event tracker;          // worker to notify on each edge change

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                ticker.periodic(1, TICK);
                break;
            case TRACK:
                tracker = take(out); // side-effect: clear "out"
                break;
            case TICK:
                out = ticked(); // may have a reply for tracking worker
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
        }

        return tracker;
    }
} rusher;

struct GpsWorker : Worker, ubx::Parser<100> {
    enum TAG { START, RECV };

    bool dump =false;

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                gpsUart.read(0, { wId, RECV });
                break;
            case RECV: {
                auto i = 0U;
                while (i < in.eVal)
                    if (parse(gpsUart.rxPtr[i++])) {
                        if (dump) {
                            if (pktClass == 0x01 && pktMsgId == 0x07) {
                                auto& pvt = *(ubx::NavPvt*) payload;
                                logf("fix %d lon %d lat %d hacc %d sv %d"
                                     " %04d-%02d-%02d %02d:%02d:%02d acc %d",
                                        pvt.fixType, pvt.lon, pvt.lat,
                                        pvt.hAcc, pvt.numSV,
                                        pvt.year, pvt.month, pvt.day,
                                        pvt.hour, pvt.min, pvt.sec, pvt.tAcc);
                            } else {
                                logf("GPS %02x %02x", pktClass, pktMsgId);
                                logDump(payload, pktLen);
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
                        blinker.enable = false;
                        rusher.ledSel = 'd';
                        break;
                    case 'm':
                        blinker.enable = false;
                        rusher.ledSel = 'm';
                        break;
                    case 'g':
                        gpser.dump = !gpser.dump;
                        break;
                    case 'l':
                        blinker.enable = !blinker.enable;
                        if (blinker.enable)
                            rusher.ledSel = 0; // stop tracking DCF or MSF
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
    rusher.init();  rusher.wName = "rush";
    gpser.init();   gpser.wName = "gps";
    blinker.init(); blinker.wName = "blink";
    cmder.init();   cmder.wName = "cmd";
    watcher.init(); watcher.wName = "watch";
    idler.init();   idler.wName = "idle";

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
