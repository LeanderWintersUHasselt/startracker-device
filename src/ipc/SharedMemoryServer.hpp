#pragma once
#include "../../shared/ipc.h"
#include <cstdint>
#include <string>

// SharedMemoryServer — backend-side shared memory manager.
// Opens /dev/shm/startracker (creates if absent, never unlinks on close so
// a frontend restart can re-mmap without reconnection logic).
// All write methods implement the seqlock protocol:
//   seq |= 1 (release) → write data → seq += 2 (release)
class SharedMemoryServer {
public:
    struct PoseData {
        double x{0}, y{0}, z{0};
        double roll{0}, pitch{0}, yaw{0};
        double pose_confidence{0};
        uint64_t timestamp_us{0};
    };

    // Opens and mmaps the shared memory file.
    // Returns true on success, false on error (check errno).
    bool open();

    // Unmaps the shared memory. Does NOT unlink — intentional.
    void close();

    // Write pose fields under the pose seqlock.
    void writePose(const PoseData& pd);

    // Write grayscale preview frame under the preview seqlock.
    // luma must point to w*h bytes of 8-bit luma. No JPEG encoding.
    void writePreview(const uint8_t* luma, uint32_t w, uint32_t h);

    // Direct access to the mapped block (for status flag writes from worker threads).
    // Caller may write status atomics directly: block()->tracking_active.store(1, relaxed)
    SharedBlock* block() { return block_; }
    const SharedBlock* block() const { return block_; }

private:
    SharedBlock* block_{nullptr};
    int          fd_{-1};
};
