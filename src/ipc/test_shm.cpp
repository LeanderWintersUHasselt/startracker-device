// Minimal test: open shared memory, write a pose via seqlock, read it back.
// Run with: g++ -std=c++17 -I. -Isrc src/ipc/test_shm.cpp src/ipc/SharedMemoryServer.cpp
//               -o /tmp/test_shm && /tmp/test_shm
#include "ipc/SharedMemoryServer.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

int main() {
    SharedMemoryServer srv;
    assert(srv.open() && "shm open failed");

    // Write a pose
    SharedMemoryServer::PoseData pd{};
    pd.x = 1.23; pd.y = 4.56; pd.z = 1.35;
    pd.roll = 0.1; pd.pitch = 0.2; pd.yaw = 45.0;
    pd.pose_confidence = 0.01;
    pd.timestamp_us = 999'000'000ULL;
    srv.writePose(pd);

    // Read back via raw block pointer
    const SharedBlock* blk = srv.block();
    uint64_t seq1 = blk->pose_seq.load(std::memory_order_acquire);
    assert((seq1 & 1) == 0 && "seqlock must be even (not writing) after writePose");
    assert(blk->x == 1.23);
    assert(blk->yaw == 45.0);

    // Write a grayscale preview frame
    std::vector<uint8_t> luma(PREVIEW_W * PREVIEW_H, 128);
    srv.writePreview(luma.data(), PREVIEW_W, PREVIEW_H);
    uint64_t pv_seq = blk->preview_seq.load(std::memory_order_acquire);
    assert((pv_seq & 1) == 0 && "preview_seq must be even after writePreview");
    assert(blk->preview_w == PREVIEW_W);
    assert(blk->preview_h == PREVIEW_H);

    srv.close();
    printf("SharedMemoryServer: all assertions passed.\n");
}
