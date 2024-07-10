// A "Mapped Romable File Store" for embedded flash memory.
#pragma once

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace mrfs {
    struct File {
        uint32_t magic, size;
        char name [15], zero;
        uint32_t time, check;

        uint8_t const* begin () const { return (uint8_t const*) (this+1); }
        uint8_t const* end () const { return begin() + size; }
    };
    static_assert(sizeof (File) == 32);

    static constexpr auto MAGIC = 0x5346524D; // 'MRFS'

    inline static File* base; // first entry in flash
    inline static File* fill; // next unused position
    inline static File* last; // past end of memory used for MRFS

    inline static File ofile; // check != 0 when in progress
    inline static uint8_t obuf [32]; // collect bytes to write in 32-byte chunks
                                     //
    inline int unused () { return (uintptr_t) last - (uintptr_t) (fill+1); }
    inline File* next (File* p) { return p + 1 + (p->size+31)/32; }

    uint32_t time10d () {
        auto dt = rtc::getDate();
        auto d = 10000*dt.yr + 100*dt.mo + dt.dy;
        auto t = 64*dt.hh + dt.mm;
        return (d<<11) + t;
    }

    void eraseRom (uintptr_t offset, uint32_t len) {
        assert(offset < (1<<21)); // offset from start of flash, not addr
        for (auto i = 0U; i < len; i += flash::pageSize(i))
            flash::erase(offset + i);
    }

    void writeRom (uintptr_t offset, void const* ptr) {
        assert(offset < (1<<21)); // offset from start of flash, not addr
        flash::write8w(offset, (uint32_t const*) ptr);
    }

    inline void init (void* ptr, uint32_t len) {
        // assert(ptr != nullptr && (uintptr_t) ptr % sizeof (uint32_t) == 0);
        base = (File*) ptr;
        fill = base;
        last = base + len / sizeof (File);
        while (fill < last && fill->magic == MAGIC)
            fill = next(fill);
    }

    inline void format () {
        fill = base;
        eraseRom((uintptr_t) base, (uintptr_t) last - (uintptr_t) base);
    }

    inline File const* open (char const* name =nullptr) {
        // an open output file always overrides any others
        if (ofile.magic != 0 && name == nullptr)
            return &ofile;

        // go over all entries, looking for the last filename match
        File const* p = nullptr;
        if (name != nullptr)
            for (auto q = base; q < fill; q = next(q))
                if (strcmp(name, q->name) == 0)
                    p = q->time != ~0U ? q : nullptr; // it may be a deletion
        return p;
    }

    inline int create (char const* name =nullptr) {
        if (name != nullptr) {
            if (ofile.magic != 0)
                return -1;
            ofile.magic = MAGIC;
            strncpy(ofile.name, name, sizeof ofile.name);
        }
        return unused();
    }

    inline int write (void const* ptr, uint32_t len) {
        if (ofile.magic == 0 || len > unused() - ofile.size)
            return -1;
        for (auto i = 0U; i < len; ++i) {
            auto pos = ofile.size % sizeof obuf;
            if (pos == 0)
                memset(obuf, 0xFF, sizeof obuf);
            obuf[pos] = ((uint8_t const*) ptr)[i];
            if (pos == sizeof obuf - 1) {
                auto p = (uintptr_t) (uint8_t*) (fill+1) + ofile.size - pos;
                writeRom(p & 0x07FF'FFFF, obuf);
            }
            ++ofile.size;
        }
        return len;
    }

    inline int close (uint32_t time =0) {
        if (ofile.magic == 0)
            return -1;
        // flush any bytes still in obuf
        if (auto pos = ofile.size % sizeof obuf; pos != 0) {
            auto p = (uintptr_t) (uint8_t*) (fill+1) + ofile.size - pos;
            writeRom(p & 0x07FF'FFFF, obuf);
        }
        if (ofile.time == 0)
            ofile.time = time != 0 ? time : time10d();
        writeRom((uintptr_t) fill & 0x07FF'FFFF, &ofile);
        fill = next(fill);
        memset(&ofile, 0, sizeof ofile);
        return 0;
    }

    inline int remove (char const* name =nullptr) {
        auto p = open(name);
        if (p == nullptr)
            return -1;
        if (p != &ofile)
            create(name);
        ofile.time = ~0;
        return close();
    }

    inline bool readDir (File*& curr) {
        curr = curr == nullptr ? base : next(curr);
        while (curr < fill && open(curr->name) != curr)
            curr = next(curr); // skip obsolete entries
        return curr < fill;
    }

    inline File const* listFiles () {
        for (File* p = nullptr; readDir(p); )
            printf("%p: %6d  %d.%04d  %s\n",
                    (void*) p, p->size,
                    20200000+p->time/10000, p->time%10000, p->name);
        return fill;
    }
}
