#include <jee.h>
using namespace jeeh;
#include "test.h"

extern "C" int _write (int, char* ptr, int len) {
    logWriter(ptr, len);
    return len;
}

int main () {
    Tester t;

    printf("Hello from %s @ %d MHz\n", SVDNAME, SystemCoreClock / 1'000'000);
}
