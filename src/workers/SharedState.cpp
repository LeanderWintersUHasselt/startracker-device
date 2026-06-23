#include "SharedState.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <ctime>
#include <filesystem>

namespace fs = std::filesystem;

static uint64_t read_realtime_us() {
    timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

static uint64_t read_monotonic_us() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

void SessionLogger::open(const char* mode_label,
                          const std::string& bin_dir,
                          const std::string& star_map_nm,
                          float calib_h,
                          float ceiling_h) {
    std::lock_guard<std::mutex> lk(mtx);
    if (fp_pose)   { std::fclose(fp_pose);   fp_pose   = nullptr; }
    if (fp_vision) { std::fclose(fp_vision); fp_vision = nullptr; }

    std::time_t now = std::time(nullptr);
    struct tm tm_buf; ::localtime_r(&now, &tm_buf);
    char dirname[64];
    std::snprintf(dirname, sizeof(dirname),
        "%04d%02d%02d_%02d%02d%02d_%s",
        tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
        tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec,
        mode_label);

    session_dir    = bin_dir + "/validation/sessions/" + dirname;
    this->mode_label = mode_label;
    star_map_name  = star_map_nm;
    calib_height   = calib_h;
    ceiling_height = ceiling_h;
    start_mono_us  = read_monotonic_us();
    start_real_us  = read_realtime_us();
    pose_rows      = 0;
    vision_rows    = 0;

    detect_samples_us.clear();
    pose_est_samples_us.clear();
    eskf_lag_samples_us.clear();
    freed_send_samples_us.clear();
    vision_interval_us.clear();
    last_vision_us = 0;

    fs::create_directories(session_dir);

    fp_pose = std::fopen((session_dir + "/pose_output.csv").c_str(), "w");
    if (fp_pose)
        std::fprintf(fp_pose,
            "timestamp_us,realtime_us,state,eskf_init,use_imu,"
            "pose_x,pose_y,pose_z,pose_roll_deg,pose_pitch_deg,pose_yaw_deg\n");

    fp_vision = std::fopen((session_dir + "/pose_vision.csv").c_str(), "w");
    if (fp_vision)
        std::fprintf(fp_vision,
            "timestamp_us,realtime_us,state,"
            "vis_x,vis_y,vis_z,vis_roll_deg,vis_pitch_deg,vis_yaw_deg,"
            "n_detections,n_inliers,reproj_err_px,match_pct,verdict\n");

    // Write initial session_meta.json (no end time yet)
    FILE* mf = std::fopen((session_dir + "/session_meta.json").c_str(), "w");
    if (mf) {
        std::fprintf(mf,
            "{\n"
            "  \"session_id\": \"%s\",\n"
            "  \"mode\": \"%s\",\n"
            "  \"start_monotonic_us\": %" PRIu64 ",\n"
            "  \"start_realtime_us\": %" PRIu64 ",\n"
            "  \"end_realtime_us\": null,\n"
            "  \"star_map\": \"%s\",\n"
            "  \"calib_height_m\": %.4f,\n"
            "  \"ceiling_height_m\": %.4f,\n"
            "  \"avg_pose_hz\": null,\n"
            "  \"avg_vision_hz\": null\n"
            "}\n",
            dirname, mode_label,
            start_mono_us, start_real_us,
            star_map_name.c_str(),
            calib_height, ceiling_height);
        std::fclose(mf);
    }

    active.store(fp_pose != nullptr, std::memory_order_release);
    std::fprintf(stderr, "[logger] Session: %s\n", session_dir.c_str());
}

void SessionLogger::close() {
    std::lock_guard<std::mutex> lk(mtx);
    active.store(false, std::memory_order_release);

    if (fp_pose)   { std::fflush(fp_pose);   std::fclose(fp_pose);   fp_pose   = nullptr; }
    if (fp_vision) { std::fflush(fp_vision); std::fclose(fp_vision); fp_vision = nullptr; }

    if (session_dir.empty()) return;

    const uint64_t end_real_us  = read_realtime_us();
    const uint64_t end_mono_us  = read_monotonic_us();
    const double   elapsed_s    = static_cast<double>(end_mono_us - start_mono_us) * 1e-6;
    const double   avg_pose_hz  = (elapsed_s > 0.0 && pose_rows > 0)
                                  ? static_cast<double>(pose_rows)   / elapsed_s : 0.0;
    const double   avg_vis_hz   = (elapsed_s > 0.0 && vision_rows > 0)
                                  ? static_cast<double>(vision_rows) / elapsed_s : 0.0;

    const std::string session_id = fs::path(session_dir).filename().string();

    FILE* mf = std::fopen((session_dir + "/session_meta.json").c_str(), "w");
    if (mf) {
        std::fprintf(mf,
            "{\n"
            "  \"session_id\": \"%s\",\n"
            "  \"mode\": \"%s\",\n"
            "  \"start_monotonic_us\": %" PRIu64 ",\n"
            "  \"start_realtime_us\": %" PRIu64 ",\n"
            "  \"end_realtime_us\": %" PRIu64 ",\n"
            "  \"star_map\": \"%s\",\n"
            "  \"calib_height_m\": %.4f,\n"
            "  \"ceiling_height_m\": %.4f,\n"
            "  \"avg_pose_hz\": %.1f,\n"
            "  \"avg_vision_hz\": %.1f,\n",
            session_id.c_str(), mode_label.c_str(),
            start_mono_us, start_real_us, end_real_us,
            star_map_name.c_str(),
            calib_height, ceiling_height,
            avg_pose_hz, avg_vis_hz);

    // ── Compute latency stats and write perf block ────────────────────────
    auto stats = [](std::vector<uint32_t>& v)
        -> std::tuple<double, double> {   // (mean_us, p95_us)
        if (v.empty()) return {0.0, 0.0};
        std::sort(v.begin(), v.end());
        double sum = 0.0;
        for (auto x : v) sum += x;
        const double mean = sum / static_cast<double>(v.size());
        const size_t idx95 = v.size() == 1
            ? 0
            : std::min(v.size() - 1,
                       static_cast<size_t>(std::ceil(0.95 * static_cast<double>(v.size()))) - 1);
        return {mean, static_cast<double>(v[idx95])};
    };

    auto [det_mean,  det_p95]  = stats(detect_samples_us);
    auto [pe_mean,   pe_p95]   = stats(pose_est_samples_us);
    auto [elag_mean, elag_p95] = stats(eskf_lag_samples_us);
    auto [fs_mean,   fs_p95]   = stats(freed_send_samples_us);

    // Vision FPS from inter-arrival times: p95 of interval → p5 of FPS
    double vis_fps_mean = 0.0, vis_fps_p5 = 0.0;
    if (!vision_interval_us.empty()) {
        std::sort(vision_interval_us.begin(), vision_interval_us.end());
        double sum = 0.0;
        for (auto x : vision_interval_us) sum += x;
        const double mean_us = sum / static_cast<double>(vision_interval_us.size());
        const size_t idx95 = vision_interval_us.size() == 1
            ? 0
            : std::min(vision_interval_us.size() - 1,
                       static_cast<size_t>(std::ceil(0.95 * static_cast<double>(vision_interval_us.size()))) - 1);
        if (mean_us > 0.0) vis_fps_mean = 1e6 / mean_us;
        if (vision_interval_us[idx95] > 0)
            vis_fps_p5 = 1e6 / static_cast<double>(vision_interval_us[idx95]);
    }

    const double pipe_mean = det_mean + pe_mean;
    const double pipe_p95  = det_p95  + pe_p95;
    const double e2e_mean  = pipe_mean + elag_mean + fs_mean;
    const double e2e_p95   = pipe_p95  + elag_p95  + fs_p95;
    const double us2ms     = 1.0 / 1000.0;

    std::fprintf(mf,
        "  \"perf\": {\n"
        "    \"vision_fps_mean\": %.1f,\n"
        "    \"vision_fps_p5\": %.1f,\n"
        "    \"detect_ms_mean\": %.2f,\n"
        "    \"detect_ms_p95\": %.2f,\n"
        "    \"pose_est_ms_mean\": %.2f,\n"
        "    \"pose_est_ms_p95\": %.2f,\n"
        "    \"pipeline_ms_mean\": %.2f,\n"
        "    \"pipeline_ms_p95\": %.2f,\n"
        "    \"eskf_lag_ms_mean\": %.2f,\n"
        "    \"eskf_lag_ms_p95\": %.2f,\n"
        "    \"freed_send_ms_mean\": %.2f,\n"
        "    \"freed_send_ms_p95\": %.2f,\n"
        "    \"end_to_end_ms_mean\": %.2f,\n"
        "    \"end_to_end_ms_p95\": %.2f\n"
        "  }\n"
        "}\n",
        vis_fps_mean, vis_fps_p5,
        det_mean  * us2ms, det_p95  * us2ms,
        pe_mean   * us2ms, pe_p95   * us2ms,
        pipe_mean * us2ms, pipe_p95 * us2ms,
        elag_mean * us2ms, elag_p95 * us2ms,
        fs_mean   * us2ms, fs_p95   * us2ms,
        e2e_mean  * us2ms, e2e_p95  * us2ms);

    // Clear vectors to free memory
    detect_samples_us.clear();    detect_samples_us.shrink_to_fit();
    pose_est_samples_us.clear();  pose_est_samples_us.shrink_to_fit();
    eskf_lag_samples_us.clear();  eskf_lag_samples_us.shrink_to_fit();
    freed_send_samples_us.clear();freed_send_samples_us.shrink_to_fit();
    vision_interval_us.clear();   vision_interval_us.shrink_to_fit();

    std::fclose(mf);
    }

    session_dir.clear();
}

void SessionLogger::row_pose(uint64_t mono_us, uint64_t real_us,
                              const char* state, bool eskf_init, bool use_imu,
                              double x, double y, double z,
                              double roll_deg, double pitch_deg, double yaw_deg) {
    if (!active.load(std::memory_order_relaxed)) return;
    std::lock_guard<std::mutex> lk(mtx);
    if (!fp_pose) return;
    std::fprintf(fp_pose,
        "%" PRIu64 ",%" PRIu64 ",%s,%d,%d,"
        "%.4f,%.4f,%.4f,%.2f,%.2f,%.2f\n",
        mono_us, real_us, state,
        eskf_init ? 1 : 0, use_imu ? 1 : 0,
        x, y, z, roll_deg, pitch_deg, yaw_deg);
    ++pose_rows;
}

void SessionLogger::row_vision(uint64_t mono_us, uint64_t real_us,
                                const char* state,
                                float x, float y, float z,
                                float roll_deg, float pitch_deg, float yaw_deg,
                                int n_det, int n_inliers,
                                float reproj_px, float match_pct,
                                const char* verdict) {
    if (!active.load(std::memory_order_relaxed)) return;
    std::lock_guard<std::mutex> lk(mtx);
    if (!fp_vision) return;
    std::fprintf(fp_vision,
        "%" PRIu64 ",%" PRIu64 ",%s,"
        "%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,"
        "%d,%d,%.2f,%.1f,%s\n",
        mono_us, real_us, state,
        x, y, z, roll_deg, pitch_deg, yaw_deg,
        n_det, n_inliers, reproj_px, match_pct, verdict);
    ++vision_rows;
}

void SessionLogger::record_latency(uint64_t capture_us,
                                    uint64_t detect_done_us,
                                    uint64_t pose_done_us) {
    if (!active.load(std::memory_order_relaxed)) return;
    std::lock_guard<std::mutex> lk(mtx);
    if (detect_done_us >= capture_us)
        detect_samples_us.push_back(
            static_cast<uint32_t>(detect_done_us - capture_us));
    if (pose_done_us >= detect_done_us)
        pose_est_samples_us.push_back(
            static_cast<uint32_t>(pose_done_us - detect_done_us));
    if (last_vision_us > 0 && pose_done_us > last_vision_us)
        vision_interval_us.push_back(
            static_cast<uint32_t>(pose_done_us - last_vision_us));
    last_vision_us = pose_done_us;
}

void SessionLogger::record_eskf_lag(uint64_t lag_us) {
    if (!active.load(std::memory_order_relaxed)) return;
    std::lock_guard<std::mutex> lk(mtx);
    eskf_lag_samples_us.push_back(static_cast<uint32_t>(lag_us));
}

void SessionLogger::record_freed_send(uint64_t send_us) {
    if (!active.load(std::memory_order_relaxed)) return;
    std::lock_guard<std::mutex> lk(mtx);
    freed_send_samples_us.push_back(static_cast<uint32_t>(send_us));
}
