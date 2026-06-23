#pragma once
// FrameSlot — lock-free ping-pong handoff from Core 1 (producer) to Core 2 (consumer).
// No heap allocation or refcounting in the hot path.
//
// Core 1 alternates writes between slot[0] and slot[1], always writing to the slot
// with the lower seq. Core 2 reads the slot with the higher even seq.
// seq is a seqlock: odd = writing, even = ready.
#include <atomic>
#include <cstdint>
#include <vector>
#include <opencv2/core.hpp>

struct FrameSlot {
    std::atomic<uint64_t>    seq{0};        // seqlock: odd=writing, even=ready
    cv::Mat                  frame;         // undistorted/preprocessed
    std::vector<cv::Point2f> undistorted;   // undistorted keypoints from Core 1
    uint64_t                 capture_us{0}; // when nextFrame() returned — before undistort/detect
    uint64_t                 timestamp_us{0}; // after detection complete
};
