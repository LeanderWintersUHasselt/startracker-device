#include "SharedMemoryServer.hpp"
#include <cerrno>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

bool SharedMemoryServer::open() {
    // Open with O_CREAT — do not unlink/recreate (frontend keeps mmap valid).
    fd_ = ::open(SHM_PATH, O_RDWR | O_CREAT, 0666);
    if (fd_ < 0) return false;

    // Ensure the file is exactly the right size.
    // ftruncate is a no-op if already the right size.
    if (::ftruncate(fd_, sizeof(SharedBlock)) != 0) {
        ::close(fd_); fd_ = -1; return false;
    }

    void* p = ::mmap(nullptr, sizeof(SharedBlock),
                     PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (p == MAP_FAILED) {
        ::close(fd_); fd_ = -1; return false;
    }
    block_ = static_cast<SharedBlock*>(p);
    return true;
}

void SharedMemoryServer::close() {
    if (block_) {
        ::munmap(block_, sizeof(SharedBlock));
        block_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    // Intentionally NOT calling shm_unlink — frontend mmap remains valid.
}

void SharedMemoryServer::writePose(const PoseData& pd) {
    // Seqlock write protocol:
    //   1. Load current seq (relaxed) — normally even; force even if odd from crash.
    //   2. Store seq|1 (release) — signals write in progress to readers.
    //   3. Write all non-atomic pose fields.
    //   4. Store seq+2 (release) — signals write complete.
    uint64_t seq = block_->pose_seq.load(std::memory_order_relaxed) & ~uint64_t(1);
    block_->pose_seq.store(seq | 1, std::memory_order_release);

    block_->x               = pd.x;
    block_->y               = pd.y;
    block_->z               = pd.z;
    block_->roll            = pd.roll;
    block_->pitch           = pd.pitch;
    block_->yaw             = pd.yaw;
    block_->pose_confidence = pd.pose_confidence;
    block_->pose_timestamp_us = pd.timestamp_us;

    block_->pose_seq.store(seq + 2, std::memory_order_release);
}

void SharedMemoryServer::writePreview(const uint8_t* bgr, uint32_t w, uint32_t h) {
    // Seqlock write protocol for BGR888 preview.
    // Caller provides raw BGR bytes (w*h*3). No JPEG encode/decode.
    uint32_t bytes = w * h * 3;
    if (bytes > PREVIEW_MAX_BYTES) bytes = PREVIEW_MAX_BYTES;

    uint64_t seq = block_->preview_seq.load(std::memory_order_relaxed) & ~uint64_t(1);
    block_->preview_seq.store(seq | 1, std::memory_order_release);

    block_->preview_w        = w;
    block_->preview_h        = h;
    block_->preview_frame_id = seq / 2 + 1;
    ::memcpy(block_->preview_bytes, bgr, bytes);

    block_->preview_seq.store(seq + 2, std::memory_order_release);
}
