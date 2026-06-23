#pragma once
// ipc.h — shared between backend (src/) and frontend (ui/)
// Included by both CMake targets via the shared/ include directory.
#include <atomic>
#include <cstdint>

constexpr uint32_t PREVIEW_W         = 1024;
constexpr uint32_t PREVIEW_H         = 576;
constexpr uint32_t PREVIEW_MAX_BYTES = PREVIEW_W * PREVIEW_H * 3;  // BGR888, 3 bytes/pixel
constexpr const char* SHM_PATH  = "/dev/shm/startracker";
constexpr const char* SOCK_PATH = "/run/startracker/startracker.sock";

// SharedBlock is mapped into /dev/shm/startracker by the backend at startup.
// The frontend mmaps the same file read-only.
// All sequence counters and status flags are std::atomic for correct ordering.
struct SharedBlock {
    // --- Pose data (Core 3 writes Kalman output at ~100 Hz) ---
    std::atomic<uint64_t> pose_seq{0};
    double    x{0}, y{0}, z{0};
    double    roll{0}, pitch{0}, yaw{0};
    double    pose_confidence{0};
    uint64_t  pose_timestamp_us{0};

    // --- Preview frame (Core 1 writes BGR888 at ~15-20 Hz) ---
    // Raw 512×288 BGR, 3 bytes/pixel — no JPEG encode/decode overhead.
    // Frontend reads as QImage::Format_BGR888.
    std::atomic<uint64_t> preview_seq{0};
    uint32_t  preview_w{PREVIEW_W};
    uint32_t  preview_h{PREVIEW_H};
    uint64_t  preview_frame_id{0};
    uint8_t   preview_bytes[PREVIEW_MAX_BYTES];

    // --- Status flags ---
    std::atomic<uint8_t>  calibration_complete{0};
    std::atomic<uint8_t>  tracking_active{0};
    std::atomic<uint8_t>  tracking_lost{0};
    std::atomic<uint8_t>  imu_ok{0};
    std::atomic<uint8_t>  freed_connected{0};
    std::atomic<float>    freed_hz{0.f};
    std::atomic<float>    freed_latency_ms{0.f};
    std::atomic<float>    vision_hz{0.f};   // rate of new camera poses arriving at Core 3
    std::atomic<float>    pose_hz{0.f};     // rate of ESKF output poses written to shm

    // --- ESKF diagnostics (Core 3 writes at ~100 Hz, UI reads for monitoring) ---
    std::atomic<float> eskf_vel_norm{0.f};  // velocity magnitude [m/s]
    std::atomic<float> eskf_ba_norm{0.f};   // accel bias norm [m/s²]
    std::atomic<float> eskf_bg_norm{0.f};   // gyro bias norm [rad/s]

};
