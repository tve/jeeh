// Main application code, combines all sorts of time-related functionality.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

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
                ++tracker.eVal; // keep track of current tick count
                out = ticked();
                // show signal on LED if enabled
                switch (ledSel) {
                    case 'd': led = dcfNow; break;
                    case 'm': led = msfNow; break;
                }
                break;
            default:
                fail();
        }
        return out;
    }

    Event ticked () {
        auto dcfPrev = dcfNow, msfPrev = msfNow;
        dcfNow = dcfDat;
        msfNow = !msfDat; // inverted signal
        if (dcfNow == dcfPrev && msfNow == msfPrev)
            return {};
        return tracker;
    }
} rusher;

struct GpsWorker : Worker {
    enum TAG { START, RECV };

    bool dump =false;

    Event process (Event in, Event out) override {
        switch (in.eTag) {
            case START:
                gps.read(0, { wId, RECV });
                break;
            case RECV:
                if (dump)
                    _write(1, (char*) gps.rxPtr, in.eVal);
                gps.read(in.eVal, { wId, RECV });
                break;
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
    gps.init(UART1_PINS, 9600);

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
