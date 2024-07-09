#include <doctest.h>
#include "mrfs.h"

#include "mrfs-arch.h"

constexpr auto N = 512;

TEST_CASE("init") {
    maprom(N);
    CHECK(mrfs::create() <= N-32);

    mrfs::format();
    CHECK(mrfs::create() == N-32);
    //CHECK(blah[0] == 0xFF);
    //CHECK(blah[N-1] == 0xFF);

    CHECK(mrfs::close() == -1);

    [[maybe_unused]] mrfs::File* fp = nullptr;
    CHECK(mrfs::readDir(fp) == false);

    CHECK(mrfs::open() == nullptr);
    CHECK(mrfs::open("abc") == nullptr);
    CHECK(mrfs::create("abc") == N-32);
    CHECK(mrfs::open("abc") == nullptr); // not (yet) found
    REQUIRE(mrfs::open() != nullptr);    // but this does find it

    // can't create more than one file at a time
    CHECK(mrfs::create("def") == -1);

    CHECK(mrfs::write("ABC", 3) == 3);
    CHECK(mrfs::open()->size == 3);
    CHECK(mrfs::write("DE\n", 3) == 3);
    CHECK(mrfs::open()->size == 6);

    CHECK(mrfs::create() == N-32);
    CHECK(mrfs::close() == 0);
    CHECK(mrfs::create() == N-3*32);

    [[maybe_unused]] auto p = mrfs::open("abc");
    REQUIRE(p != nullptr);
    CHECK(p->size == 6);
    CHECK(memcmp(p->begin(), "ABCDE\n", 6) == 0);

    CHECK(mrfs::open() == nullptr);
    CHECK(mrfs::open("def") == nullptr);

    CHECK(mrfs::create("def") == N-3*32);
    CHECK(mrfs::close() == 0);
    CHECK(mrfs::create() == N-4*32);

    CHECK(mrfs::create("ghi") == N-4*32);
    CHECK(mrfs::write("XYZ", 3) == 3);
    CHECK(mrfs::close() == 0);
    CHECK(mrfs::create() == N-6*32);

    REQUIRE(mrfs::open("abc") != nullptr);
    REQUIRE(mrfs::open("def") != nullptr);
    REQUIRE(mrfs::open("ghi") != nullptr);

    CHECK(mrfs::open("abc")->size == 6);
    CHECK(mrfs::open("def")->size == 0);
    CHECK(mrfs::open("ghi")->size == 3);
}

TEST_CASE("re-init") {
    maprom(N);

    REQUIRE(mrfs::open("abc") != nullptr);
    REQUIRE(mrfs::open("def") != nullptr);
    REQUIRE(mrfs::open("ghi") != nullptr);

    CHECK(mrfs::open("abc")->size == 6);
    CHECK(mrfs::open("def")->size == 0);
    CHECK(mrfs::open("ghi")->size == 3);
}

TEST_CASE("remove") {
    maprom(N);

    REQUIRE(mrfs::open("abc") != nullptr);
    REQUIRE(mrfs::open("def") != nullptr);
    REQUIRE(mrfs::open("ghi") != nullptr);

    CHECK(mrfs::remove("abc") == 0);
    CHECK(mrfs::open("abc") == nullptr);

    CHECK(mrfs::open("def")->size == 0);
    CHECK(mrfs::open("ghi")->size == 3);

    CHECK(mrfs::create("jkl") == N-7*32);
    CHECK(mrfs::close() == 0);
    CHECK(mrfs::create() == N-8*32);

    CHECK(mrfs::create("abc") == N-8*32);
    CHECK(mrfs::write("IJKL", 4) == 4);
    CHECK(mrfs::close() == 0);
    CHECK(mrfs::create() == N-10*32);
}

TEST_CASE("verify") {
    maprom(N);

    REQUIRE(mrfs::open("abc") != nullptr);
    REQUIRE(mrfs::open("def") != nullptr);
    REQUIRE(mrfs::open("ghi") != nullptr);
    REQUIRE(mrfs::open("jkl") != nullptr);

    CHECK(mrfs::open("abc")->size == 4);
    CHECK(mrfs::open("def")->size == 0);
    CHECK(mrfs::open("ghi")->size == 3);
    CHECK(mrfs::open("jkl")->size == 0);

    CHECK(memcmp(mrfs::open("abc")->begin(), "IJKL", 4) == 0);
    CHECK(memcmp(mrfs::open("ghi")->begin(), "XYZL", 3) == 0);
}

TEST_CASE("listFiles") {
    maprom(N);
    mrfs::listFiles();
}

TEST_CASE("extend") {
    maprom(N);
    mrfs::format();

    CHECK(mrfs::create("hundred") == N-32);
    for (int i = 0; i < 10; ++i)
        CHECK(mrfs::write("1234567890", 10));
    CHECK(mrfs::close() == 0);
    CHECK(mrfs::create() == N-6*32);

    CHECK(mrfs::create("zero") == N-6*32);
    CHECK(mrfs::close() == 0);
    CHECK(mrfs::create() == N-7*32);

    mrfs::listFiles();
}
