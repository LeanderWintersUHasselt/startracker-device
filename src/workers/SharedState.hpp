#pragma once

#include "common/Config.hpp"
#include "common/Types.hpp"
#include "localiser/StarIndex.hpp"
#include "localiser/StarIndex3D.hpp"
#include "pipeline/FrameSlot.hpp"
#include "pose/PoseEstimator.hpp"
#include "util/Intrinsics.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ── Inter-worker channels ─────────────────────────────────────────────────────

// Ping-pong frame handoff Core1→Core2. Access as slots[0] and slots[1].
using FrameSlots = std::array<FrameSlot, 2>;

struct CameraMeas {
    std::mutex            mtx;
    CameraPoseMeasurement meas;
    uint64_t              version{0};
};

struct VisionPose {
    std::mutex mtx;
    PoseResult result;
    uint64_t   version{0};
};

// ── Map state ─────────────────────────────────────────────────────────────────
// Written by handle_command (and main() at boot), read by TrackingWorker.
// generation is incremented each time a new map is installed — TrackingWorker
// watches it and reconstructs Localiser3D/Tracker3D on change.

struct MapState {
    StarMap                      star_map;
    std::unique_ptr<StarIndex>   star_index;
    StarMap3D                    star_map3d;
    std::unique_ptr<StarIndex3D> star_index3d;
    std::string                  star_map_name;
    std::atomic<bool>            map_ready{false};
    std::atomic<bool>            map3d_ready{false};
    std::atomic<uint32_t>        generation{0};
};

// ── Shared config (mutable at runtime via set_config socket command) ──────────
// Workers snapshot fields they need under the lock at the start of each loop.

struct SharedConfig {
    std::mutex mtx;
    AppConfig  cfg;
};

// ── Runtime settings (persisted to ~/.config/startracker/settings.json) ───────
// Consolidates former RuntimeSettings + EskfRuntimeSettings into one struct.

struct RuntimeSettings {
    std::string freed_ip      = "192.168.1.100";
    int         freed_port    = 40000;
    bool        freed_enabled = true;
    bool        imu_enabled   = true;
    float       sigma_cam_pos   = 0.05f;
    float       sigma_cam_att   = 0.01f;
    float       vel_decay_s     = 0.0f;
    float       smooth_min_cutoff = 0.0f;
    float       smooth_beta       = 0.5f;
    double      camera_offset_x_m = 0.0;
    double      camera_offset_y_m = 0.0;
    double      camera_offset_z_m = 0.0;
};

struct SharedSettings {
    std::mutex      mtx;
    RuntimeSettings data;
};

// ── IMU debug settings (mutable at runtime via set_imu_debug command) ─────────
// handle_command writes; IMUWorker snapshots each iteration.

struct ImuDebugSettings {
    bool        enabled      = true;
    bool        skip_predict = false;
    std::string perturb_axis = "none";
    double      perturb_deg  = 0.0;
};

struct SharedImuDebug {
    std::mutex       mtx;
    ImuDebugSettings data;
};

// ── Session CSV logger ────────────────────────────────────────────────────────
// handle_command calls open()/close(). IMUWorker calls row_pose(); TrackingWorker calls row_vision().

struct SessionLogger {
    std::mutex          mtx;
    FILE*               fp_pose   = nullptr;
    FILE*               fp_vision = nullptr;
    std::atomic<bool>   active{false};

    // Set at open(), read at close()
    std::string  session_dir;
    std::string  mode_label;
    std::string  star_map_name;
    float        calib_height   = 0.f;
    float        ceiling_height = 0.f;
    uint64_t     start_mono_us  = 0;
    uint64_t     start_real_us  = 0;
    uint64_t     pose_rows      = 0;   // guarded by mtx
    uint64_t     vision_rows    = 0;   // guarded by mtx

    // ── Latency accumulators (μs, one entry per event, cleared at open()) ──────
    std::vector<uint32_t> detect_samples_us;       // nextFrame→detection done
    std::vector<uint32_t> pose_est_samples_us;     // detection done→pose accepted
    std::vector<uint32_t> eskf_lag_samples_us;     // detection done→ESKF update
    std::vector<uint32_t> freed_send_samples_us;   // freed.send() call duration
    std::vector<uint32_t> vision_interval_us;      // inter-arrival times (for FPS)
    uint64_t              last_vision_us{0};       // last pose_done_us

    void open(const char* mode_label,
              const std::string& bin_dir,
              const std::string& star_map_nm,
              float calib_h,
              float ceiling_h);
    void close();

    void row_pose(uint64_t mono_us, uint64_t real_us,
                  const char* state, bool eskf_init, bool use_imu,
                  double x, double y, double z,
                  double roll_deg, double pitch_deg, double yaw_deg);

    void row_vision(uint64_t mono_us, uint64_t real_us,
                    const char* state,
                    float x, float y, float z,
                    float roll_deg, float pitch_deg, float yaw_deg,
                    int n_det, int n_inliers,
                    float reproj_px, float match_pct,
                    const char* verdict);

    // ── Latency recording (called from worker threads) ─────────────────────────
    void record_latency(uint64_t capture_us, uint64_t detect_done_us, uint64_t pose_done_us);
    void record_eskf_lag(uint64_t lag_us);
    void record_freed_send(uint64_t send_us);
};

// ── Preview debug mode ────────────────────────────────────────────────────────
// Stored as int in ControlState::preview_debug_mode; this enum names the values.

enum class PreviewDebugMode : int { Off = 0, Normal = 1, Light = 2 };

// ── Control state (cross-cutting signals between workers and handle_command) ──

enum class TrackState : int { Idle = 0, Localise = 1, Tracking = 2 };

struct ControlState {
    // TrackingWorker writes; CameraWorker + handle_command read
    std::atomic<int>   track_state{static_cast<int>(TrackState::Idle)};
    // Camera pause handshake: handle_command sets cam_pause_req; CameraWorker signals cam_paused
    std::atomic<bool>  cam_pause_req{false};
    std::atomic<bool>  cam_paused{false};
    // handle_command-internal signals used by calibration and map-build threads
    std::atomic<bool>  calib_confirm{false};
    std::atomic<bool>  build_map_stop{false};
    // handle_command writes; IMUWorker reads
    std::atomic<bool>  imu_only_mode{false};
    // handle_command writes; CameraWorker reads
    std::atomic<int>   preview_debug_mode{0};
    std::atomic<bool>  preview_grid{true};
    // handle_command writes (set_config); TrackingWorker reads
    std::atomic<int>   min_tracking_stars{3};
    // Calibration heights: set at startup + calibration; read by TrackingWorker + IMUWorker
    std::atomic<float> calib_height{1.399f};
    std::atomic<float> ceiling_height_m{2.4f};
    // Camera intrinsics: written at startup by main(), then immutable — no lock needed
    Intrinsics         intr;
    // Config file path: written at startup; read by handle_command for saveConfig calls
    std::string        config_path;
    // Session logger: handle_command calls open/close; IMUWorker calls row()
    SessionLogger      logger;
};
