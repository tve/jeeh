// Common code, used in all tests.

#include <unity.h>
#include <jee.h>
#include <jee/cycles.h>
#include <jee/dma.h>
using namespace jeeh;

#undef assert
#define assert TEST_ASSERT

// tie printf and logf into Unity's output mechanism

extern "C" int _write (int, char* ptr, int len) {
    for (auto i = 0; i < len; ++i)
        putchar(ptr[i]);
    return len;
}

void jeeh::logWriter (void const* ptr, size_t len) {
    _write(1, (char*) ptr, len);
}

extern void allTests ();

int main () {
    fastClock();
    cycles::init();

    UNITY_BEGIN();
    allTests();
    return UNITY_END();
}

// End of boilerplate, below is for temporary code, to be moved once ready.
