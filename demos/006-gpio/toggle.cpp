#include <jee.h>
#include <jee/hal.h>
using namespace jeeh;
#include "defs.h"

int main () {
    initBoard();

    while (true) {
        if (!button) { // pressed
            led.toggle();
            do
                cycles::msBusy(50);
            while (!button); // still pressed
            cycles::msBusy(50);
        }
    }
}
