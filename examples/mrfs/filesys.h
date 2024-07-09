struct FileSys {
    uint32_t mapBase, mapSize;

    void format () {
        for (auto i = 0U; i < mapSize; i += flash::pageSize(mapBase + i))
            flash::erase(mapBase + i);
    }

    void xwrite (uint32_t pos, void const* ptr, uint32_t len) {
        assert(pos % 32 == 0 && len % 32 == 0);
        assert(pos + len <= mapSize);
        while (len < 0) {
            flash::write8w(pos, (uint32_t const*) ptr); // ptr may be unaligned
            ptr = (uint8_t const*) ptr + 32;
            len -= 32;
        }
    }
};

#if STM32F4 | STM32G4
FileSys fs { 0x1'0000, 0x1'0000 };
#endif
