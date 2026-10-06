#pragma once
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

namespace shmmw {

// Thin RAII wrapper around a POSIX shared memory segment. This is the whole
// point of the project: blocks are separate OS processes that talk through
// shm_open()/mmap(), never through a socket. create() is called by exactly
// one process per segment name (the graph launcher, before any block execs);
// every block process calls attach() and never create(), so there is no
// runtime race to "who creates the segment".
class ShmSegment {
public:
    ShmSegment() = default;
    ~ShmSegment() { close_(); }

    ShmSegment(const ShmSegment&) = delete;
    ShmSegment& operator=(const ShmSegment&) = delete;

    ShmSegment(ShmSegment&& other) noexcept { *this = std::move(other); }
    ShmSegment& operator=(ShmSegment&& other) noexcept {
        if (this != &other) {
            close_();
            addr_ = other.addr_; size_ = other.size_; name_ = std::move(other.name_); owner_ = other.owner_;
            other.addr_ = nullptr; other.size_ = 0; other.owner_ = false;
        }
        return *this;
    }

    static ShmSegment create(const std::string& name, size_t size) {
        ShmSegment s;
        shm_unlink(name.c_str()); // best effort, ignore ENOENT from a stale run
        int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
        if (fd < 0) {
            std::fprintf(stderr, "shm_open(create) failed for %s: %s\n", name.c_str(), std::strerror(errno));
            std::exit(1);
        }
        if (ftruncate(fd, static_cast<off_t>(size)) != 0) {
            std::fprintf(stderr, "ftruncate failed for %s: %s\n", name.c_str(), std::strerror(errno));
            std::exit(1);
        }
        s.map_(fd, size, name, true, PROT_READ | PROT_WRITE);
        return s;
    }

    static ShmSegment attach(const std::string& name, size_t size) {
        ShmSegment s;
        int fd = -1;
        // The launcher execs block processes immediately after creating every
        // segment, but exec() and the child's first shm_open() are not
        // synchronized by anything except that ordering, so a genuinely fast
        // child can lose a shm_open race against a slow filesystem on the
        // very first attempt. Retry briefly rather than treating one failed
        // open as a fatal error; this was found by a flaky first test run
        // (see README, "what broke").
        for (int attempt = 0; attempt < 200 && fd < 0; ++attempt) {
            fd = shm_open(name.c_str(), O_RDWR, 0600);
            if (fd < 0) usleep(1000);
        }
        if (fd < 0) {
            std::fprintf(stderr, "shm_open(attach) failed for %s: %s\n", name.c_str(), std::strerror(errno));
            std::exit(1);
        }
        s.map_(fd, size, name, false, PROT_READ | PROT_WRITE);
        return s;
    }

    // Attaches with PROT_READ only: the mapping itself cannot be written to,
    // so "does not touch producer state" is enforced by the kernel's page
    // protection, not by convention. Used by `snooper`, which has no business
    // reason to ever hold a writable mapping to a ring it does not own. See
    // tests/test_snooper.cpp for a test that proves a write through this
    // mapping faults.
    static ShmSegment attach_readonly(const std::string& name, size_t size) {
        ShmSegment s;
        int fd = -1;
        for (int attempt = 0; attempt < 200 && fd < 0; ++attempt) {
            fd = shm_open(name.c_str(), O_RDONLY, 0600);
            if (fd < 0) usleep(1000);
        }
        if (fd < 0) {
            std::fprintf(stderr, "shm_open(attach_readonly) failed for %s: %s\n", name.c_str(), std::strerror(errno));
            std::exit(1);
        }
        s.map_(fd, size, name, false, PROT_READ);
        return s;
    }

    void* data() const { return addr_; }
    size_t size() const { return size_; }

private:
    void map_(int fd, size_t size, const std::string& name, bool owner, int prot) {
        void* addr = mmap(nullptr, size, prot, MAP_SHARED, fd, 0);
        ::close(fd);
        if (addr == MAP_FAILED) {
            std::fprintf(stderr, "mmap failed for %s: %s\n", name.c_str(), std::strerror(errno));
            std::exit(1);
        }
        addr_ = addr;
        size_ = size;
        name_ = name;
        owner_ = owner;
    }

    void close_() {
        if (addr_) munmap(addr_, size_);
        if (owner_ && !name_.empty()) shm_unlink(name_.c_str());
        addr_ = nullptr;
    }

    void* addr_ = nullptr;
    size_t size_ = 0;
    std::string name_;
    bool owner_ = false;
};

} // namespace shmmw
