#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace one_cycle {
struct FrameStats {
    uint64_t frames{}, bytes{};
    bool valid{};
};

// Read-only counter ABI from the installed IgH CLI's DWARF, not the MCU or PDO ABI.
class FrameCounter {
    struct Module { uint32_t magic, masters; };
    using Master = std::array<unsigned char, DCU_IGH_MASTER_INFO_SIZE>;
    static_assert(DCU_IGH_TX_COUNT_OFFSET + sizeof(uint64_t) <= sizeof(Master));
    static_assert(DCU_IGH_TX_BYTES_OFFSET + sizeof(uint64_t) <= sizeof(Master));
    int fd_;
public:
    FrameCounter() : fd_(open("/dev/EtherCAT0", O_RDONLY | O_CLOEXEC))
    {
        Module module{};
        if (fd_ < 0 || ioctl(fd_, _IOR(0xa4, 0, Module), &module) != 0 ||
            module.magic != DCU_IGH_IOCTL_MAGIC || module.masters != 1) {
            if (fd_ >= 0) close(fd_);
            throw std::runtime_error("unexpected IgH master counter ABI");
        }
    }
    FrameCounter(const FrameCounter &) = delete;
    FrameCounter &operator=(const FrameCounter &) = delete;
    ~FrameCounter() { close(fd_); }

    FrameStats read() const
    {
        Master master;
        FrameStats s{};
        if (ioctl(fd_, _IOR(0xa4, 1, Master), &master) == 0) {
            std::memcpy(&s.frames, master.data() + DCU_IGH_TX_COUNT_OFFSET, sizeof(s.frames));
            std::memcpy(&s.bytes, master.data() + DCU_IGH_TX_BYTES_OFFSET, sizeof(s.bytes));
            s.valid = true;
        }
        return s;
    }
};
}
