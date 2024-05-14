// Demonstrate thread locks with the "dining philosophers problem".
// See https://rosettacode.org/wiki/Dining_philosophers#Python

#include <jee.h>
using namespace jeeh;
#include "test.h"

constexpr auto N = 5;  // number of philosophers
constexpr auto M = 6;  // how often they want to eat

char const* names [] = {
    "Aristotle", "Kant", "Spinoza", "Marx", "Russell"
};
static_assert(sizeof names / sizeof *names == N);

Lock forks [N];

int philo (Message& m) {
    auto id = m.mTag;
    assert(1 <= id && id <= N);
    auto name = names[id-1];

    auto& forkOnLeft = forks[id-1];
    auto& forkOnRight = forks[id%N];

    for (auto i = 0; i < M; ++i) {
        sys::wait(30 + rand() % 100);
        logf("%s is hungry", name);

        auto f1 = &forkOnLeft, f2 = &forkOnRight;
        while (true) {
            f1->acquire(true);
            if (f2->acquire(false))
                break;
            f1->release();
            logf("%s swaps forks", name);
            swap(f1, f2);
        }

        logf("%s starts eating", name);
        sys::wait(10 + rand() % 90);
        logf("%s finishes eating and leaves to think", name);

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
        assert(id == i+1);
    }

    for (auto i = 0; i < N; ++i)
        sys::recv(); // wait for child thread completion
}
