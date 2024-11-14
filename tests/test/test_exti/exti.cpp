// External pin interrupt tests.

#include "../common.h"
#include "jee/ticker.h"
#include <jee/exti.h>

Ticker ticker;
TICKER_INSTALL(ticker)

ExtIrq exti;
EXTIRQ_INSTALL(exti)

void setUp () {}
void tearDown () {}

void testJumper () {
    Pin outPin ("A9","P"), inPin ("A10","F");

    // check that the two pins are connected via a jumper
    TEST_ASSERT_EQUAL(0, inPin);
    outPin = 1;
    TEST_ASSERT_EQUAL(1, inPin);
}

struct ExtIinterrupt : Task {
    enum TAG { START, ONE, TWO, THREE, FOUR };

    Pin outPin;
    uint16_t start;
    char capture [20];
    uint8_t calls =0;
    bool done =false;

    ExtIinterrupt () : Task ("ExtIinterrupt"), outPin ("A9","P") {}

    Event process (Event in, Event out) override {
        capture[calls++] = '0' + in.eTag;
        TEST_ASSERT_LESS_OR_EQUAL(sizeof capture, calls+1); // trailing zero

        switch (in.eTag) {
            case START:
                start = ticker.millis();
                exti.enable((Pin) "A10", exti.BOTH, TWO);
                ticker.periodic(4, ONE); // toggle the output pin
                ticker.delay(20, THREE); // disable the exti input
                ticker.delay(30, FOUR);  // end of test
                break;
            case ONE:
                outPin.toggle();
                break;
            case TWO:
                break;
            case THREE:
                exti.disable((Pin) "A10");
                break;
            case FOUR:
                ticker.cancel(ONE); // cancel periodic timer
                capture[calls] = 0;
                done = true;
                break;
            default:
                fail();
        }
        return out;
    }
};

void testExti () {
    ExtIinterrupt task;
    auto tkId = ticker.init();
    auto exId = exti.init();
    auto wkId = task.init();

    TEST_ASSERT_GREATER_THAN(0, wkId);
    TEST_ASSERT_GREATER_THAN(wkId, exId);
    TEST_ASSERT_GREATER_THAN(exId, tkId);

    // start 3 delays and set up an EXTI pin interrupt
    // FIXME Task::send({ wkId, task.START });

    int n = 0;
    do { asm ("wfi"); ++n; } while (!task.done);

    TEST_ASSERT_EQUAL_STRING("01212121231114", task.capture);

    // since the ticker runs every 1 ms, there will have been 30 interrupts
    TEST_ASSERT_EQUAL(30, n);
    Task::showStats();
}

void allTests () {
    RUN_TEST(testJumper);
    RUN_TEST(testExti);
}
