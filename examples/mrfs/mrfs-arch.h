#if __ARM_ARCH && !__ARM_PCS_VFP && !__APPLE__ // arm cortex ...

#include <jee.h>

uint32_t mrfs::time10d () {
    auto dt = jeeh::rtc::get();
    auto d = 10000*(dt.yr-20) + 100*dt.mo + dt.dy;
    auto t = 100*dt.hh + dt.mm;
    return 10000*d + t; // 10-digit int: YYMMDDhhmm, relative to 2020
}

void mrfs::eraseRom (uintptr_t rom, uint32_t len) {
    for (auto i = 0U; i < len; i += jeeh::flash::pageSize(i))
        jeeh::flash::erasePage(rom + i);
}

void mrfs::writeRom (uintptr_t dest, void const* ptr) {
    jeeh::flash::write32b(dest, ptr);
}

#else // not arm cortex, i.e. native code

#include <cassert>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>

uint32_t mrfs::time10d () {
    auto now = time(nullptr);
    auto tm = localtime(&now);
    auto d = 10000*(tm->tm_year%100-20) + 100*(tm->tm_mon+1) + tm->tm_mday;
    auto t = 100*tm->tm_hour + tm->tm_min;
    return 10000*d + t; // 10-digit int: YYMMDDhhmm, relative to 2020
}

void mrfs::eraseRom (uintptr_t rom, uint32_t len) {
    memset((void*) rom, 0xFF, len);
}

void mrfs::writeRom (uintptr_t dest, void const* ptr) {
    memcpy((void*) dest, ptr, 32);
}

void* maprom (uint32_t size, char const* name =nullptr) {
    if (name == nullptr)
        name = "rom.dat";
    auto fd = open(name, O_CREAT|O_RDWR, 0666); assert(fd > 0);
    auto e = ftruncate(fd, size); assert(e == 0);
    auto ptr = mmap(nullptr, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    assert(ptr != MAP_FAILED);
    mrfs::init(ptr, size);
    return ptr;
}

#endif // __ARM_ARCH && !__ARM_PCS_VFP !__APPLE__
