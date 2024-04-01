// Demonstrate thread locks with the "dining philosophers problem".
// See https://rosettacode.org/wiki/Dining_philosophers#Python

#include <jee.h>
using namespace jeeh;
#include "test.h"

constexpr auto N = 5;   // number of philosophers
constexpr auto M = 10;  // how often they want to eat

Lock forks [N];

int philo (Message& m) {
sys::wait(1);
    auto id = m.mTag;
logf("s %p %d", &m, id);
return 0;
    assert(1 <= id && id <= N);

    auto& forkOnLeft = forks[id-1];
    auto& forkOnRight = forks[id%N];

    for (auto i = 0; i < M; ++i) {
        sys::wait(30 + rand() % 100);
        logf("%d is hungry", id);

        auto f1 = &forkOnLeft, f2 = &forkOnRight;
        while (true) {
            f1->acquire(true);
            if (f2->acquire(false))
                break;
            f1->release();
            logf("%d swaps forks", id);
            swap(f1, f2);
        }

        logf("%d starts eating", id);
        sys::wait(10 + rand() % 90);
        logf("%d finishes eating and leaves to think", id);

        f2->release();
        f1->release();
    }

    return 0;
}

int main () {
    Tester t;

    uint32_t stack [300];
    sys::init(stack);

    uint32_t philoStack [N][200];

    for (auto i = 0; i < N; ++i) {
        auto id = sys::fork(philoStack[i], philo).mTag;
logf("S %p %d", philoStack[i], id);
        assert(id == i+1);
    }

    for (auto i = 0; i < N; ++i)
        sys::recv(); // wait for child thread completion
}
