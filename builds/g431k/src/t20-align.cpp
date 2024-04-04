// Use the watchdog to reset the system after a while.

#include <jee.h>
using namespace jeeh;
#include "test.h"

struct Empty {};
struct Basic { char x; };
struct Array { char x [2]; };

int main () {
    Tester t;

    logf("     char %2d %% %d b", sizeof (char),      alignof (char));
    logf("    short %2d %% %d b", sizeof (short),     alignof (short));
    logf("      int %2d %% %d b", sizeof (int),       alignof (int));
    logf("     long %2d %% %d b", sizeof (long),      alignof (long));
    logf("long long %2d %% %d b", sizeof (long long), alignof (long long));
    logf("    float %2d %% %d b", sizeof (float),     alignof (float));
    logf("   double %2d %% %d b", sizeof (double),    alignof (double));
    logf("    void* %2d %% %d b", sizeof (void*),     alignof (void*));
    logf("  Empty{} %2d %% %d b", sizeof (Empty),     alignof (Empty));
    logf("  Basic{} %2d %% %d b", sizeof (Basic),     alignof (Basic));
    logf("  Array{} %2d %% %d b", sizeof (Array),     alignof (Array));
    logf("");
    logf("  Message %2d %% %d b", sizeof (Message),   alignof (Message));
    logf("    Chain %2d %% %d b", sizeof (Chain),     alignof (Chain));
    logf("     Task %2d %% %d b", sizeof (Task),      alignof (Task));
    logf("    Fixer %2d %% %d b", sizeof (Fixer),     alignof (Fixer));
    logf("     Lock %2d %% %d b", sizeof (Lock),      alignof (Lock));
    logf("   Device %2d %% %d b", sizeof (Device),    alignof (Device));
    logf(" DateTime %2d %% %d b", sizeof (DateTime),  alignof (DateTime));
    logf("      Pin %2d %% %d b", sizeof (Pin),       alignof (Pin));
}
