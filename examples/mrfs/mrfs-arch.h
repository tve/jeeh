// flash- and architecture-specific code

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
