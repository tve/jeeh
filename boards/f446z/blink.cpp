#include <jee.h>
using namespace jeeh;

static void delayLoop (int n) {
    for (int i = 0; i < n * 3000; ++i)
        asm ("");
}

int main () {
    const Pin led1 ("B0","P");
    const Pin led2 ("B7","P");
    const Pin led3 ("B14","P");

    while (true) {
        led1.toggle();
        led2.toggle();
        led3.toggle();
        delayLoop(500);
    }
}
