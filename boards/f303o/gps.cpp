// Bidirectional bridge between the tty and GPS serial ports.

#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

struct Pool {
    enum { N=25 };
    uint8_t const* ring [N];
    uint8_t inPos =0, outPos =0;

    bool add (uint8_t const* data, uint8_t size) {
        assert(size > 0);
        auto buf = (uint8_t*) malloc(1+size);
        buf[0] = size;
        memcpy(buf+1, data, size);

        bool wasEmpty = inPos == outPos;

        ring[inPos] = buf;
        inPos = (inPos+1) % N;
        assert(inPos != outPos); // overflow

        return wasEmpty;
    }

    uint8_t const* next () const {
        return outPos != inPos ? ring[outPos] : nullptr;
    }

    bool remove () {
        assert(outPos != inPos);
        free((void*) ring[outPos]);
        outPos = (outPos+1) % N;
        return outPos != inPos;
    }
};

struct Echo : Task {
    enum TAG { START, GPS_RX, GPS_TX, TTY_RX, TTY_TX };

    Pool gpsPool, ttyPool;
    bool gpsBusy =false, ttyBusy =false;

    Event process (Event in, Event out) override {
        //logf("t %d v %d", in.eTag, in.eVal);
        switch (in.eTag) {
            case START:
                gpsUart.setReply(GPS_RX);
                gpsUart.read(nullptr, 0);
                ttyUart.setReply(TTY_RX);
                ttyUart.read(nullptr, 0);
                break;

            case GPS_RX: // gps data received
                if (ttyPool.add(gpsUart.rxPtr, in.eVal))
                    ttyBusy = feed(ttyPool, ttyUart, TTY_TX);
                gpsUart.setReply(GPS_RX);
                gpsUart.read(nullptr, in.eVal);
                break;

            case TTY_TX: // ttyUart data send done
                if (ttyPool.remove())
                    ttyBusy = feed(ttyPool, ttyUart, TTY_TX);
                break;

            case TTY_RX: // tty data received
                if (gpsPool.add(ttyUart.rxPtr, in.eVal))
                    gpsBusy = feed(gpsPool, gpsUart, GPS_TX);
                ttyUart.setReply(TTY_RX);
                ttyUart.read(nullptr, in.eVal);
                break;

            case GPS_TX: // gps data send done
                if (gpsPool.remove())
                    gpsBusy = feed(gpsPool, gpsUart, GPS_TX);
                break;

            default:
                fail();
        }
        return out;
    }

    template< typename UART >
    bool feed (Pool& pool, UART& txUart, TAG done) {
        auto buf = pool.next();
        if (buf == nullptr)
            return false;
        txUart.setReply(done);
        txUart.write(buf+1, *buf);
        return true;
    }
};

int main () {
    initBoard();
    gpsUart.init(9600);

    Echo echo;
    echo.init();

    while (true)
        led1 = +gpsPps;
}
