#if __ARM_ARCH && !__ARM_PCS_VFP && !__APPLE__ // arm cortex ...

#include <jee.h>

uint32_t mrfs::time10d () {
    auto dt = rtc::getDate();
    auto d = 10000*dt.yr + 100*dt.mo + dt.dy;
    auto t = 64*dt.hh + dt.mm;
    return (d<<11) + t;
}

void mrfs::eraseRom (uintptr_t offset, uint32_t len) {
    assert(offset < (1<<21)); // offset from start of flash, not addr
    for (auto i = 0U; i < len; i += flash::pageSize(i))
        flash::erase(offset + i);
}

void mrfs::writeRom (uintptr_t offset, void const* ptr) {
    assert(offset < (1<<21)); // offset from start of flash, not addr
    flash::write8w(offset, (uint32_t const*) ptr);
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
