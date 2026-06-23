/*
 * startracker daemon — Plan B + Plan C
 *
 * Multi-threaded background daemon. No terminal UI.
 * Communicates with the Qt6 frontend via:
 *   - /dev/shm/startracker  (SharedBlock, seqlock)
 *   - /run/startracker/startracker.sock  (Unix socket, JSON newline-delimited)
 *
 * Thread layout:
 *   Main thread  : SocketServer event loop + command dispatch
 *   Core 1 thread: Camera capture → undistort → StarDetectorLight → grayscale preview → shm
 *   Core 2 thread: FrameSlot (ping-pong) → localise/track → PoseData → shm + vis_pose
 *   Core 3 thread: ImuReader (BNO085) + ESKF propagate/correct + FreeD D1 UDP output
 */

#include "common/Config.hpp"
#include "common/Types.hpp"
#include "detector/StarDetector.hpp"
#include "detector/StarDetectorLight.hpp"
#include "geometry/Calibration.hpp"
#include "geometry/Transform.hpp"
#include "io/CameraReader.hpp"
#include "ipc/FreeD.hpp"
#include "ipc/SharedMemoryServer.hpp"
#include "ipc/SocketServer.hpp"
#include "localiser/Localiser3D.hpp"
#include "localiser/StarIndex.hpp"
#include "localiser/StarIndex3D.hpp"
#include "mapper/MapBuilder.hpp"
#include "pipeline/FrameSlot.hpp"
#include "pose/PoseEstimator.hpp"
#include "tracker/Tracker3D.hpp"
#include "util/ConfigLoader.hpp"
#include "util/Intrinsics.hpp"
#include "workers/SharedState.hpp"
#include "workers/CameraWorker.hpp"
#include "workers/IMUWorker.hpp"
#include "workers/TrackingWorker.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <pthread.h>
#include <signal.h>
#include <string>
#include <thread>
#include <sys/stat.h>
#include <unistd.h>

namespace fs = std::filesystem;
using clk = std::chrono::steady_clock;

// ── Thread affinity helper ────────────────────────────────────────────────────
// Default: SCHED_OTHER + CPU affinity. Do NOT use SCHED_FIFO for CV workers.
// SCHED_FIFO on heavy CV threads increases jitter, not throughput on Pi 5.
// Only consider SCHED_FIFO for the output thread (Core 3) AFTER profiling shows
// outbound FreeD jitter that cannot be solved any other way.
// Note: pthread_setaffinity_np is Linux-only.

void pin_to_core(int core) {
#ifdef __linux__
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)
        std::perror("pthread_setaffinity_np");
    // pthread_setschedparam: leave at SCHED_OTHER (default)
#else
    (void)core;  // CPU affinity not supported on this platform
#endif
}

// ── CLOCK_MONOTONIC microseconds ──────────────────────────────────────────────

uint64_t now_us() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

// ── Minimal JSON field extractors (no nlohmann dependency) ───────────────────

static std::string json_str(const std::string& line, const std::string& key) {
    std::string needle = "\"" + key + "\"";
    auto pos = line.find(needle);
    if (pos == std::string::npos) return {};
    pos += needle.size();
    while (pos < line.size() && (line[pos]==' '||line[pos]==':')) ++pos;
    if (pos >= line.size() || line[pos] != '"') return {};
    ++pos;
    std::string result;
    while (pos < line.size() && line[pos] != '"') {
        if (line[pos]=='\\' && pos+1<line.size()) ++pos;
        result += line[pos++];
    }
    return result;
}

static double json_dbl(const std::string& line, const std::string& key,
                       double def = 0.0) {
    std::string needle = "\"" + key + "\"";
    auto pos = line.find(needle);
    if (pos == std::string::npos) return def;
    pos += needle.size();
    while (pos < line.size() && (line[pos]==' '||line[pos]==':')) ++pos;
    if (pos >= line.size()) return def;
    try { return std::stod(line.substr(pos)); }
    catch (...) { return def; }
}

static int json_int(const std::string& line, const std::string& key,
                    int def = 0) {
    return static_cast<int>(json_dbl(line, key, def));
}

static bool json_bool(const std::string& line, const std::string& key,
                      bool def = false) {
    std::string needle = "\"" + key + "\"";
    auto pos = line.find(needle);
    if (pos == std::string::npos) return def;
    pos += needle.size();
    while (pos < line.size() && (line[pos] == ' ' || line[pos] == ':')) ++pos;
    if (pos >= line.size()) return def;
    if (line.compare(pos, 4, "true")  == 0) return true;
    if (line.compare(pos, 5, "false") == 0) return false;
    if (line[pos] == '1') return true;
    if (line[pos] == '0') return false;
    return def;
}

static void write_preview_from_bgr(SharedMemoryServer* shm, const cv::Mat& bgr) {
    if (!shm || bgr.empty()) return;
    cv::Mat small;
    cv::resize(bgr, small, cv::Size(PREVIEW_W, PREVIEW_H), 0, 0, cv::INTER_AREA);
    if (small.channels() == 1) cv::cvtColor(small, small, cv::COLOR_GRAY2BGR);
    shm->writePreview(small.data, PREVIEW_W, PREVIEW_H);
}


static PreviewDebugMode preview_debug_mode_from_string(const std::string& mode) {
    if (mode == "normal") return PreviewDebugMode::Normal;
    if (mode == "light")  return PreviewDebugMode::Light;
    return PreviewDebugMode::Off;
}

static const char* preview_debug_mode_to_string(PreviewDebugMode mode) {
    switch (mode) {
        case PreviewDebugMode::Normal: return "normal";
        case PreviewDebugMode::Light:  return "light";
        case PreviewDebugMode::Off:
        default: return "off";
    }
}

void write_preview_fit_from_mat(SharedMemoryServer* shm, const cv::Mat& frame) {
    if (!shm || frame.empty()) return;
    cv::Mat bgr;
    if (frame.channels() == 1) cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    else bgr = frame;

    cv::Mat canvas(PREVIEW_H, PREVIEW_W, CV_8UC3, cv::Scalar(0, 0, 0));
    const double sx = static_cast<double>(PREVIEW_W) / bgr.cols;
    const double sy = static_cast<double>(PREVIEW_H) / bgr.rows;
    const double scale = std::min(sx, sy);
    const int outW = std::max(1, static_cast<int>(std::lround(bgr.cols * scale)));
    const int outH = std::max(1, static_cast<int>(std::lround(bgr.rows * scale)));

    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(outW, outH), 0, 0,
               scale < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
    const int offX = (PREVIEW_W - outW) / 2;
    const int offY = (PREVIEW_H - outH) / 2;
    resized.copyTo(canvas(cv::Rect(offX, offY, outW, outH)));
    shm->writePreview(canvas.data, PREVIEW_W, PREVIEW_H);
}

static std::string json_points_normalized(const std::vector<cv::Point2f>& pts,
                                          float width,
                                          float height,
                                          size_t limit = 256) {
    if (pts.empty() || width <= 1.f || height <= 1.f) return "[]";
    std::string out = "[";
    const size_t n = std::min(limit, pts.size());
    char buf[64];
    for (size_t i = 0; i < n; ++i) {
        float nx = std::clamp(pts[i].x / width, 0.f, 1.f);
        float ny = std::clamp(pts[i].y / height, 0.f, 1.f);
        std::snprintf(buf, sizeof(buf), "[%.5f,%.5f]", nx, ny);
        if (i) out += ',';
        out += buf;
    }
    out += ']';
    return out;
}

// ── Settings (persisted to ~/.config/startracker/settings.json) ──────────────
// SharedSettings is written by the main thread (socket command handlers) and
// read by Core 3 (IMUWorker). Protect with SharedSettings.mtx.
// RuntimeSettings is defined in workers/SharedState.hpp.

static fs::path settings_path() {
    const char* home = std::getenv("HOME");
    return fs::path(home ? home : "/root") /
           ".config" / "startracker" / "settings.json";
}

static RuntimeSettings load_settings() {
    RuntimeSettings s;
    auto p = settings_path();
    std::ifstream f(p);
    if (!f) return s;
    std::string content((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());
    if (auto v = json_str(content, "freed_ip"); !v.empty()) s.freed_ip = v;
    s.freed_port    = json_int (content, "freed_port",    s.freed_port);
    s.freed_enabled = json_bool(content, "freed_enabled", s.freed_enabled);
    s.imu_enabled   = json_bool(content, "imu_enabled",   s.imu_enabled);
    s.sigma_cam_pos = static_cast<float>(json_dbl(content, "sigma_cam_pos", s.sigma_cam_pos));
    s.sigma_cam_att = static_cast<float>(json_dbl(content, "sigma_cam_att", s.sigma_cam_att));
    s.vel_decay_s   = static_cast<float>(json_dbl(content, "vel_decay_s",   s.vel_decay_s));
    s.camera_offset_x_m = json_dbl(content, "camera_offset_x_m", s.camera_offset_x_m);
    s.camera_offset_y_m = json_dbl(content, "camera_offset_y_m", s.camera_offset_y_m);
    s.camera_offset_z_m = json_dbl(content, "camera_offset_z_m", s.camera_offset_z_m);
    return s;
}

static void save_settings(const RuntimeSettings& s) {
    auto p = settings_path();
    fs::create_directories(p.parent_path());
    FILE* f = std::fopen(p.c_str(), "w");
    if (!f) return;
    std::time_t now = std::time(nullptr);
    char date[16]; std::strftime(date, sizeof(date), "%Y-%m-%d", std::localtime(&now));
    std::fprintf(f,
        "{\n"
        "  \"freed_ip\": \"%s\",\n"
        "  \"freed_port\": %d,\n"
        "  \"freed_enabled\": %s,\n"
        "  \"imu_enabled\": %s,\n"
        "  \"sigma_cam_pos\": %.4f,\n"
        "  \"sigma_cam_att\": %.4f,\n"
        "  \"vel_decay_s\": %.4f,\n"
        "  \"camera_offset_x_m\": %.6f,\n"
        "  \"camera_offset_y_m\": %.6f,\n"
        "  \"camera_offset_z_m\": %.6f,\n"
        "  \"saved\": \"%s\"\n"
        "}\n",
        s.freed_ip.c_str(), s.freed_port,
        s.freed_enabled ? "true" : "false",
        s.imu_enabled ? "true" : "false",
        s.sigma_cam_pos, s.sigma_cam_att, s.vel_decay_s,
        s.camera_offset_x_m, s.camera_offset_y_m, s.camera_offset_z_m,
        date);
    std::fclose(f);
}

// ── Global shared state ───────────────────────────────────────────────────────

std::atomic<bool> g_running{true};

static bool install_star_map3d_from_legacy(const StarMap& map, MapState& map_state) {
    if (map.empty()) {
        map_state.star_map3d.clear();
        map_state.star_index3d.reset();
        map_state.map3d_ready.store(false, std::memory_order_release);
        return false;
    }

    try {
        auto map3d = util::starMap3DFromLegacy(map);
        map_state.star_map3d = map3d;
        map_state.star_index3d = std::make_unique<StarIndex3D>(
            StarIndex3D::build(map_state.star_map3d));
        map_state.map3d_ready.store(true, std::memory_order_release);
        return true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[daemon] Warning: StarMap3D fallback failed: %s\n",
                     e.what());
        map_state.star_map3d.clear();
        map_state.star_index3d.reset();
        map_state.map3d_ready.store(false, std::memory_order_release);
        return false;
    }
}

// ── Convert CameraPoseMeasurement → PoseResult for the shared-memory / FreeD path.
// T_world_cam.t = (x, y, z) where z is negative (camera below ceiling Z=0).
// Extracts full 6DOF from the PnP result: position + ZYX euler from R_world_cam.
static std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"':  out += "\\\""; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:   out += c;      break;
        }
    }
    return out;
}

static std::string config_json(const AppConfig& cfg) {
    const auto& c  = cfg.camera;
    const auto& d  = cfg.detector;
    const auto& dl = cfg.detectorLight;
    const auto& l  = cfg.localiser;
    const auto& t  = cfg.tracker;
    const auto& m  = cfg.mapper;

    char buf[8192];
    std::snprintf(buf, sizeof(buf),
        "{\"event\":\"config\",\"values\":{"
        "\"width\":%d,"
        "\"height\":%d,"
        "\"camera\":%d,"
        "\"fps\":%d,"
        "\"shutter\":%d,"
        "\"gain\":%.4f,"
        "\"lens_position\":%.4f,"
        "\"awb_gain_r\":%.4f,"
        "\"awb_gain_b\":%.4f,"
        "\"sat_max\":%d,"
        "\"val_min\":%d,"
        "\"bg_kernel\":%d,"
        "\"peak_floor\":%d,"
        "\"morph_kernel\":%d,"
        "\"area_min\":%d,"
        "\"area_max\":%d,"
        "\"circ_min\":%.4f,"
        "\"light_downsample\":%d,"
        "\"light_blur_kernel\":%d,"
        "\"light_threshold\":%d,"
        "\"light_area_min\":%d,"
        "\"light_area_max\":%d,"
        "\"light_circ_min\":%.4f,"
        "\"ransac_iter\":%d,"
        "\"thresh_px\":%.4f,"
        "\"verbose\":%s,"
        "\"max_px\":%.4f,"
        "\"max_rot_deg\":%.4f,"
        "\"min_match_pct\":%.4f,"
        "\"max_reproj_m\":%.6f,"
        "\"min_tracking_stars\":%d,"
        "\"map_fps\":%d,"
        "\"merge_radius_px\":%.4f,"
        "\"min_frame_count\":%d,"
        "\"max_new_per_frame\":%d,"
        "\"min_inlier_ratio\":%.4f,"
        "\"calib_height\":%.6f,"
        "\"star_map\":\"%s\","
        "\"intrinsics\":\"%s\","
        "\"intrinsics_backup\":\"%s\","
        "\"intrinsics_fixed\":\"%s\","
        "\"eskf_noise_scale\":%.4f,"
        "\"eskf_sigma_imu_att\":%.6f,"
        "\"eskf_sigma_cam_pos\":%.6f,"
        "\"ceiling_height_m\":%.2f,"
        "\"eskf_sigma_cam_att\":%.6f,"
        "\"vel_decay_s\":%.3f,"
        "\"smooth_min_cutoff\":%.3f,"
        "\"smooth_beta\":%.3f,"
        "\"use_heavy_detector\":%s"
        "}}",
        c.width, c.height, c.camera, c.fps, c.shutter, c.gain,
        c.lensPosition, c.awbGainR, c.awbGainB,
        d.satMax, d.valMin, d.bgKernel, d.peakFloor, d.morphKernel,
        d.areaMin, d.areaMax, d.circMin,
        dl.downsample, dl.blurKernel, dl.threshold,
        dl.areaMin, dl.areaMax, dl.circMin,
        l.ransacIter, l.threshPx, l.verbose ? "true" : "false",
        t.maxPx, t.maxRotDeg, t.minMatchPct, t.maxReprojM, t.minStars,
        m.mapFps, m.mergeRadiusPx, m.minFrameCount,
        m.maxNewPerFrame, m.minInlierRatio,
        cfg.calibHeight,
        json_escape(cfg.starMapPath).c_str(),
        json_escape(cfg.intrinsics.activePath).c_str(),
        json_escape(cfg.intrinsics.backupPath).c_str(),
        json_escape(cfg.intrinsics.fixedPath).c_str(),
        cfg.eskfNoiseScale,
        cfg.eskfSigmaImuAtt,
        cfg.eskfSigmaCamPos,
        cfg.ceilingHeightM,
        cfg.eskfSigmaCamAtt,
        cfg.eskfVelDecayS,
        cfg.smoothMinCutoff,
        cfg.smoothBeta,
        cfg.useHeavyDetector ? "true" : "false");
    return buf;
}

static void apply_config_json(const std::string& raw, AppConfig& cfg) {
    cfg.camera.width        = json_int(raw, "width", cfg.camera.width);
    cfg.camera.height       = json_int(raw, "height", cfg.camera.height);
    cfg.camera.camera       = json_int(raw, "camera", cfg.camera.camera);
    cfg.camera.fps          = json_int(raw, "fps", cfg.camera.fps);
    cfg.camera.shutter      = json_int(raw, "shutter", cfg.camera.shutter);
    cfg.camera.gain         = static_cast<float>(json_dbl(raw, "gain", cfg.camera.gain));
    cfg.camera.lensPosition = static_cast<float>(json_dbl(raw, "lens_position", cfg.camera.lensPosition));
    cfg.camera.awbGainR     = static_cast<float>(json_dbl(raw, "awb_gain_r", cfg.camera.awbGainR));
    cfg.camera.awbGainB     = static_cast<float>(json_dbl(raw, "awb_gain_b", cfg.camera.awbGainB));

    cfg.detector.satMax      = json_int(raw, "sat_max", cfg.detector.satMax);
    cfg.detector.valMin      = json_int(raw, "val_min", cfg.detector.valMin);
    cfg.detector.bgKernel    = json_int(raw, "bg_kernel", cfg.detector.bgKernel);
    cfg.detector.peakFloor   = json_int(raw, "peak_floor", cfg.detector.peakFloor);
    cfg.detector.morphKernel = json_int(raw, "morph_kernel", cfg.detector.morphKernel);
    cfg.detector.areaMin     = json_int(raw, "area_min", cfg.detector.areaMin);
    cfg.detector.areaMax     = json_int(raw, "area_max", cfg.detector.areaMax);
    cfg.detector.circMin     = static_cast<float>(json_dbl(raw, "circ_min", cfg.detector.circMin));

    cfg.detectorLight.downsample = json_int(raw, "light_downsample", cfg.detectorLight.downsample);
    cfg.detectorLight.blurKernel = json_int(raw, "light_blur_kernel", cfg.detectorLight.blurKernel);
    cfg.detectorLight.threshold  = json_int(raw, "light_threshold", cfg.detectorLight.threshold);
    cfg.detectorLight.areaMin    = json_int(raw, "light_area_min", cfg.detectorLight.areaMin);
    cfg.detectorLight.areaMax    = json_int(raw, "light_area_max", cfg.detectorLight.areaMax);
    cfg.detectorLight.circMin    = static_cast<float>(json_dbl(raw, "light_circ_min", cfg.detectorLight.circMin));

    cfg.localiser.ransacIter = json_int(raw, "ransac_iter", cfg.localiser.ransacIter);
    cfg.localiser.threshPx   = static_cast<float>(json_dbl(raw, "thresh_px", cfg.localiser.threshPx));
    cfg.localiser.verbose    = json_bool(raw, "verbose", cfg.localiser.verbose);

    cfg.tracker.maxPx       = static_cast<float>(json_dbl(raw, "max_px", cfg.tracker.maxPx));
    cfg.tracker.maxRotDeg   = static_cast<float>(json_dbl(raw, "max_rot_deg", cfg.tracker.maxRotDeg));
    cfg.tracker.minMatchPct = static_cast<float>(json_dbl(raw, "min_match_pct", cfg.tracker.minMatchPct));
    cfg.tracker.maxReprojM  = static_cast<float>(json_dbl(raw, "max_reproj_m", cfg.tracker.maxReprojM));
    cfg.tracker.minStars    = std::max(3, json_int(raw, "min_tracking_stars", cfg.tracker.minStars));

    cfg.mapper.mapFps         = json_int(raw, "map_fps", cfg.mapper.mapFps);
    cfg.mapper.mergeRadiusPx  = static_cast<float>(json_dbl(raw, "merge_radius_px", cfg.mapper.mergeRadiusPx));
    cfg.mapper.minFrameCount  = json_int(raw, "min_frame_count", cfg.mapper.minFrameCount);
    cfg.mapper.maxNewPerFrame = json_int(raw, "max_new_per_frame", cfg.mapper.maxNewPerFrame);
    cfg.mapper.minInlierRatio = static_cast<float>(json_dbl(raw, "min_inlier_ratio", cfg.mapper.minInlierRatio));

    cfg.calibHeight = static_cast<float>(json_dbl(raw, "calib_height", cfg.calibHeight));
    if (auto v = json_str(raw, "star_map"); !v.empty()) cfg.starMapPath = v;
    if (auto v = json_str(raw, "intrinsics"); !v.empty()) cfg.intrinsics.activePath = v;
    if (auto v = json_str(raw, "intrinsics_backup"); !v.empty()) cfg.intrinsics.backupPath = v;
    if (auto v = json_str(raw, "intrinsics_fixed"); !v.empty()) cfg.intrinsics.fixedPath = v;

    cfg.eskfNoiseScale   = static_cast<float>(json_dbl(raw, "eskf_noise_scale",   cfg.eskfNoiseScale));
    cfg.eskfSigmaImuAtt  = static_cast<float>(json_dbl(raw, "eskf_sigma_imu_att", cfg.eskfSigmaImuAtt));
    cfg.eskfSigmaCamPos  = static_cast<float>(json_dbl(raw, "eskf_sigma_cam_pos", cfg.eskfSigmaCamPos));
    cfg.ceilingHeightM   = static_cast<float>(json_dbl(raw, "ceiling_height_m", cfg.ceilingHeightM));
    cfg.eskfSigmaCamAtt  = static_cast<float>(json_dbl(raw, "eskf_sigma_cam_att", cfg.eskfSigmaCamAtt));
    cfg.eskfVelDecayS    = static_cast<float>(json_dbl(raw, "vel_decay_s",       cfg.eskfVelDecayS));
    cfg.smoothMinCutoff  = static_cast<float>(json_dbl(raw, "smooth_min_cutoff", cfg.smoothMinCutoff));
    cfg.smoothBeta       = static_cast<float>(json_dbl(raw, "smooth_beta",       cfg.smoothBeta));
    cfg.useHeavyDetector = json_bool(raw, "use_heavy_detector", cfg.useHeavyDetector);
}


// ── Signal handler ────────────────────────────────────────────────────────────

static void on_signal(int) { g_running = false; }

// ── File listing helpers ──────────────────────────────────────────────────────

// Returns JSON array string for scale_calibration/*.json files.
static std::string list_scale_files(const std::string& bin_dir) {
    std::string arr = "[";
    bool first = true;
    std::string dir = bin_dir + "/scale_calibration";
    try {
        std::vector<fs::path> paths;
        for (const auto& e : fs::directory_iterator(dir))
            if (e.path().extension() == ".json") paths.push_back(e.path());
        std::sort(paths.begin(), paths.end());
        for (const auto& p : paths) {
            std::ifstream f(p);
            std::string c((std::istreambuf_iterator<char>(f)),
                           std::istreambuf_iterator<char>());
            double h = json_dbl(c, "height_m", 0.0);
            auto date_val = json_str(c, "date");
            if (h <= 0) continue;
            if (!first) arr += ",";
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "{\"name\":\"%s\",\"date\":\"%s\",\"height_m\":%.4f}",
                p.filename().c_str(),
                date_val.empty() ? "unknown" : date_val.c_str(), h);
            arr += buf;
            first = false;
        }
    } catch (...) {}
    return arr + "]";
}

static double map_height_hint_from_name(const std::string& name) {
    // Handles both star_map_1.500m.csv and star_map_1.500m_YYYYMMDD_HHMMSS.csv
    for (size_t i = 1; i < name.size(); ++i) {
        if (name[i] != 'm') continue;
        char next = (i + 1 < name.size()) ? name[i + 1] : 0;
        if (next != '_' && next != '.') continue;
        size_t start = name.rfind('_', i - 1);
        start = (start == std::string::npos) ? 0 : start + 1;
        std::string num = name.substr(start, i - start);
        if (num.empty()) continue;
        bool valid = true;
        for (char c : num) if (!std::isdigit(static_cast<unsigned char>(c)) && c != '.') { valid = false; break; }
        if (!valid) continue;
        try { return std::stod(num); } catch (...) { continue; }
    }
    return 0.0;
}

// Returns JSON array string for star_maps/*.csv files.
static std::string list_star_map_files(const std::string& bin_dir) {
    std::string arr = "[";
    bool first = true;
    std::string dir = bin_dir + "/star_maps";
    try {
        std::vector<fs::path> paths;
        for (const auto& e : fs::directory_iterator(dir)) {
            if (e.path().extension() != ".csv") continue;
            std::string stem = e.path().stem().string();
            if (stem.size() >= 7 && stem.substr(stem.size() - 7) == "_metric")
                continue;
            paths.push_back(e.path());
        }
        std::sort(paths.begin(), paths.end());
        for (const auto& p : paths) {
            int stars = 0;
            bool metric_header = false;
            std::ifstream f(p);
            std::string line;
            if (std::getline(f, line))
                metric_header = line.find("x_m") != std::string::npos;
            while (std::getline(f, line))
                if (!line.empty()) ++stars;

            auto ftime  = fs::last_write_time(p);
            auto sctp   = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                              ftime - fs::file_time_type::clock::now()
                                    + std::chrono::system_clock::now());
            std::time_t tt = std::chrono::system_clock::to_time_t(sctp);
            char date[16]; std::strftime(date, sizeof(date), "%Y-%m-%d",
                                         std::localtime(&tt));

            std::string base = p.string().substr(0, p.string().rfind('.'));
            bool anchored = fs::exists(base + "_anchors.json");
            double scale_height_m = map_height_hint_from_name(p.filename().string());
            const char* scale_status = anchored ? "final" : (metric_header ? "temporary" : "unknown");
            const char* scale_label = anchored ? "anker" : (metric_header ? "tijdelijk" : "onbekend");
            const char* scale_source = anchored ? "anker" : (scale_height_m > 0.0 ? "hoogteschatting" : "niet gekoppeld");

            if (!first) arr += ",";
            char buf[512];
            std::snprintf(buf, sizeof(buf),
                "{\"name\":\"%s\",\"date\":\"%s\",\"stars\":%d,"
                "\"scale_status\":\"%s\",\"scale_label\":\"%s\","
                "\"scale_source\":\"%s\",\"scale_height_m\":%.4f}",
                p.filename().c_str(), date, stars,
                scale_status, scale_label, scale_source, scale_height_m);
            arr += buf;
            first = false;
        }
    } catch (...) {}
    return arr + "]";
}

static std::string current_star_map_json(const MapState& map_state) {
    std::string name = map_state.star_map_name;
    std::string out = "{\"event\":\"star_map\",\"name\":\"" + name + "\",\"points\":[";
    bool first = true;
    for (size_t i = 0; i < map_state.star_map.size(); ++i) {
        if (!first) out += ",";
        char item[128];
        std::snprintf(item, sizeof(item), "{\"id\":%zu,\"x\":%.6f,\"y\":%.6f}",
                      i, map_state.star_map[i].x, map_state.star_map[i].y);
        out += item;
        first = false;
    }
    out += "]}";
    return out;
}

static void persist_current_map_choice(const std::string& map_name,
                                        AppConfig& cfg,
                                        const ControlState& ctrl) {
    if (map_name.empty() || ctrl.config_path.empty()) return;
    cfg.starMapPath = "star_maps/" + map_name;
    cfg.calibHeight = ctrl.calib_height.load(std::memory_order_relaxed);
    try {
        util::saveConfig(ctrl.config_path, cfg);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[daemon] Warning: config save failed after map change: %s\n", e.what());
    }
}

// ── handle_command ────────────────────────────────────────────────────────────

static void handle_command(const std::string& cmd, const std::string& raw,
                            SocketServer::Sender send,
                            SharedMemoryServer* p_shm,
                            const std::string* p_bin_dir,
                            const std::string* p_log_root,
                            SharedConfig*       p_shared_cfg,
                            SharedSettings*     p_settings,
                            SharedImuDebug*     p_imu_debug,
                            MapState*           p_map,
                            ControlState*       p_ctrl,
                            FrameSlots*         p_slots,
                            CameraMeas*         p_cam_meas = nullptr)
{
    SharedMemoryServer& shm        = *p_shm;
    const std::string&  bin_dir    = *p_bin_dir;
    const std::string&  log_root   = *p_log_root;
    AppConfig&          cfg        = p_shared_cfg->cfg;  // direct access (command thread only)
    SharedSettings&     settings   = *p_settings;
    SharedImuDebug&     imu_debug  = *p_imu_debug;
    MapState&           map        = *p_map;
    ControlState&       ctrl       = *p_ctrl;

    if (cmd == "get_status") {
        char buf[640];
        std::string fip;
        int fport;
        bool fe, ie;
        double off_x, off_y, off_z;
        {
            std::lock_guard<std::mutex> lk(settings.mtx);
            fip   = settings.data.freed_ip;
            fport = settings.data.freed_port;
            fe    = settings.data.freed_enabled;
            ie    = settings.data.imu_enabled;
            off_x = settings.data.camera_offset_x_m;
            off_y = settings.data.camera_offset_y_m;
            off_z = settings.data.camera_offset_z_m;
        }
        std::snprintf(buf, sizeof(buf),
            "{\"event\":\"status\","
            "\"calibration_complete\":%s,"
            "\"tracking_active\":%s,"
            "\"imu_only_active\":%s,"
            "\"freed_ip\":\"%s\","
            "\"freed_port\":%d,"
            "\"freed_enabled\":%s,"
            "\"imu_enabled\":%s,"
            "\"camera_offset_x_m\":%.6f,"
            "\"camera_offset_y_m\":%.6f,"
            "\"camera_offset_z_m\":%.6f,"
            "\"detector_debug_mode\":\"%s\"}",
            shm.block()->calibration_complete.load() ? "true" : "false",
            shm.block()->tracking_active.load()      ? "true" : "false",
            ctrl.imu_only_mode.load(std::memory_order_relaxed) ? "true" : "false",
            fip.c_str(), fport,
            fe ? "true" : "false",
            ie ? "true" : "false",
            off_x, off_y, off_z,
            preview_debug_mode_to_string(static_cast<PreviewDebugMode>(
                ctrl.preview_debug_mode.load(std::memory_order_relaxed))));
        send(buf);

    } else if (cmd == "get_config") {
        send(config_json(cfg));

    } else if (cmd == "set_config") {
        {
            std::lock_guard<std::mutex> lk(p_shared_cfg->mtx);
            apply_config_json(raw, cfg);
            cfg.tracker.minStars = std::max(3, cfg.tracker.minStars);
        }
        ctrl.min_tracking_stars.store(cfg.tracker.minStars, std::memory_order_relaxed);
        ctrl.calib_height.store(cfg.calibHeight, std::memory_order_relaxed);
        ctrl.ceiling_height_m.store(cfg.ceilingHeightM, std::memory_order_relaxed);
        util::saveConfig(ctrl.config_path, cfg);
        // Sync ESKF sigma/decay from updated config into runtime settings so IMUWorker
        // sees the change immediately (mirrors fix branch set_config sync).
        {
            std::lock_guard<std::mutex> lk(settings.mtx);
            settings.data.sigma_cam_pos     = cfg.eskfSigmaCamPos;
            settings.data.sigma_cam_att     = cfg.eskfSigmaCamAtt;
            settings.data.vel_decay_s       = cfg.eskfVelDecayS;
            settings.data.smooth_min_cutoff = cfg.smoothMinCutoff;
            settings.data.smooth_beta       = cfg.smoothBeta;
        }
        // Reconstruct Localiser/Tracker on the next Core 2 loop so tracker
        // thresholds and min-stars changes apply without a daemon restart.
        map.generation.fetch_add(1, std::memory_order_release);
        send(config_json(cfg));

    } else if (cmd == "list_files") {
        std::string type = json_str(raw, "type");
        if (type == "scale_calibration") {
            char buf[4096];
            std::snprintf(buf, sizeof(buf),
                "{\"event\":\"file_list\",\"type\":\"scale_calibration\","
                "\"files\":%s}",
                list_scale_files(bin_dir).c_str());
            send(buf);
        } else if (type == "star_maps") {
            char buf[4096];
            std::snprintf(buf, sizeof(buf),
                "{\"event\":\"file_list\",\"type\":\"star_maps\","
                "\"files\":%s}",
                list_star_map_files(bin_dir).c_str());
            send(buf);
        } else {
            send("{\"event\":\"error\",\"msg\":\"unknown list_files type\"}");
        }

    } else if (cmd == "get_current_map") {
        send(current_star_map_json(map));

    } else if (cmd == "delete_map") {
        std::string file = json_str(raw, "file");
        if (file.find('/') != std::string::npos || file.find("..") != std::string::npos
                || file.size() < 5) {
            send("{\"event\":\"error\",\"msg\":\"invalid map name\"}");
            return;
        }
        std::string path = bin_dir + "/star_maps/" + file;
        std::string base = path.substr(0, path.size() - 4); // strip .csv
        fs::remove(path);
        fs::remove(base + "_metric.csv");
        fs::remove(base + "_anchors.json");
        if (map.star_map_name == file) {
            map.star_map_name.clear();
            map.star_map.clear();
            map.map_ready.store(false, std::memory_order_release);
        }
        char buf[4096];
        std::snprintf(buf, sizeof(buf),
            "{\"event\":\"file_list\",\"type\":\"star_maps\","
            "\"files\":%s}",
            list_star_map_files(bin_dir).c_str());
        send(buf);

    } else if (cmd == "calibrate_scale") {
        // Run scale calibration in a detached thread
        // (SocketServer command handler must not block).
        std::string file = json_str(raw, "file");
        std::thread([p_shm, p_bin_dir, p_ctrl, p_shared_cfg, file, send]() mutable {
            SharedMemoryServer& shm = *p_shm;
            const std::string& bin_dir = *p_bin_dir;
            ControlState& ctrl = *p_ctrl;
            AppConfig& cfg = p_shared_cfg->cfg;
            if (!file.empty()) {
                // Load from existing file
                std::string path = bin_dir + "/scale_calibration/" + file;
                std::ifstream f(path);
                if (!f) {
                    send("{\"event\":\"error\",\"msg\":\"file not found\"}");
                    return;
                }
                std::string c((std::istreambuf_iterator<char>(f)),
                               std::istreambuf_iterator<char>());
                double h = json_dbl(c, "height_m", 0.0);
                if (h <= 0) {
                    send("{\"event\":\"error\",\"msg\":\"invalid calibration file\"}");
                    return;
                }
                ctrl.calib_height.store(static_cast<float>(h));
                char buf[256];
                std::snprintf(buf, sizeof(buf),
                    "{\"event\":\"scale_done\",\"file\":\"%s\",\"height_m\":%.4f}",
                    file.c_str(), h);
                send(buf);
            } else {
                // Run new measurement.
                // Pause CameraWorker first so it releases rpicam-vid.
                ctrl.cam_pause_req.store(true, std::memory_order_release);
                auto pauseDeadline = std::chrono::steady_clock::now()
                                  + std::chrono::seconds{10};
                while (!ctrl.cam_paused.load(std::memory_order_acquire)
                       && std::chrono::steady_clock::now() < pauseDeadline)
                    std::this_thread::sleep_for(std::chrono::milliseconds{100});

                if (!ctrl.cam_paused.load(std::memory_order_acquire)) {
                    ctrl.cam_pause_req.store(false, std::memory_order_release);
                    send("{\"event\":\"error\",\"msg\":\"camera niet vrijgegeven\"}");
                    return;
                }

                // waitFn: block until frontend sends "calib_confirm" or daemon stops
                auto waitFn = [&]() -> bool {
                    ctrl.calib_confirm.store(false, std::memory_order_release);
                    while (g_running.load()
                           && !ctrl.calib_confirm.load(std::memory_order_acquire))
                        std::this_thread::sleep_for(std::chrono::milliseconds{100});
                    return g_running.load();
                };

                MapBuilder builder(ctrl.intr, cfg.mapper, cfg.detector,
                                   cfg.camera, ctrl.calib_height.load());
                auto res = builder.calibrateHeight(0.10f, waitFn, send);

                // Always release camera back to CameraWorker
                ctrl.cam_pause_req.store(false, std::memory_order_release);

                if (!res.ok) {
                    // error already sent by calibrateHeight via sendFn
                    return;
                }
                ctrl.calib_height.store(res.height_m);
                // Save JSON file
                char jname[64];
                std::snprintf(jname, sizeof(jname), "%.3fm.json", res.height_m);
                std::string jpath = bin_dir + "/scale_calibration/" + jname;
                fs::create_directories(bin_dir + "/scale_calibration");
                FILE* jf = std::fopen(jpath.c_str(), "w");
                if (jf) {
                    std::time_t now = std::time(nullptr);
                    char date[16];
                    std::strftime(date, sizeof(date), "%Y-%m-%d",
                                  std::localtime(&now));
                    std::fprintf(jf,
                        "{\n"
                        "  \"height_m\": %.4f,\n"
                        "  \"scale_px_per_m\": %.2f,\n"
                        "  \"fx\": %.2f,\n"
                        "  \"date\": \"%s\"\n"
                        "}\n",
                        res.height_m, res.scale_px_per_m,
                        ctrl.intr.fx(), date);
                    std::fclose(jf);
                }
                char buf[256];
                std::snprintf(buf, sizeof(buf),
                    "{\"event\":\"scale_done\",\"file\":\"%s\","
                    "\"height_m\":%.4f}",
                    jname, res.height_m);
                send(buf);
            }
        }).detach();

    } else if (cmd == "build_map") {
        std::string file = json_str(raw, "file");
        std::thread([p_shm, p_bin_dir, p_ctrl, p_shared_cfg, p_map, file, send]() mutable {
            SharedMemoryServer& shm = *p_shm;
            const std::string& bin_dir = *p_bin_dir;
            ControlState& ctrl = *p_ctrl;
            AppConfig& cfg = p_shared_cfg->cfg;
            MapState& map = *p_map;
            StarMap new_map;
            if (!file.empty()) {
                std::string path = bin_dir + "/star_maps/" + file;
                try {
                    new_map = util::loadStarMap(path);
                } catch (const std::exception& e) {
                    send(std::string("{\"event\":\"error\",\"msg\":\"") +
                         e.what() + "\"}");
                    return;
                }
                ctrl.track_state.store(static_cast<int>(TrackState::Idle));
                map.star_map   = new_map;
                map.star_map_name = file;
                map.star_index = std::make_unique<StarIndex>(
                                   StarIndex::build(map.star_map));
                bool loaded3d = false;
                // Try to load companion _metric.csv for PnP pipeline
                {
                    std::string base3d = path.substr(0, path.rfind('.'));
                    std::string path3d = base3d + "_metric.csv";
                    if (fs::exists(path3d)) {
                        try {
                            auto map3d = util::loadStarMap3D(path3d);
                            if (!map3d.empty()) {
                                map.star_map3d = map3d;
                                map.star_index3d = std::make_unique<StarIndex3D>(
                                    StarIndex3D::build(map3d));
                                map.map3d_ready.store(true,
                                    std::memory_order_release);
                                loaded3d = true;
                            }
                        } catch (...) {}
                    }
                }
                if (!loaded3d)
                    install_star_map3d_from_legacy(map.star_map, map);
                map.generation.fetch_add(1, std::memory_order_release);
                map.map_ready.store(true, std::memory_order_release);
                shm.block()->calibration_complete.store(1,
                    std::memory_order_relaxed);
                ctrl.track_state.store(static_cast<int>(TrackState::Localise));
                persist_current_map_choice(file, cfg, ctrl);
                char buf[256];
                std::snprintf(buf, sizeof(buf),
                    "{\"event\":\"calibration_done\","
                    "\"file\":\"%s\",\"stars\":%zu}",
                    file.c_str(), new_map.size());
                send(buf);
                send(current_star_map_json(map));
            } else {
                ctrl.cam_pause_req.store(true, std::memory_order_release);
                auto pauseDeadline = std::chrono::steady_clock::now()
                                  + std::chrono::seconds{10};
                while (!ctrl.cam_paused.load(std::memory_order_acquire)
                       && std::chrono::steady_clock::now() < pauseDeadline)
                    std::this_thread::sleep_for(std::chrono::milliseconds{100});

                if (!ctrl.cam_paused.load(std::memory_order_acquire)) {
                    ctrl.cam_pause_req.store(false, std::memory_order_release);
                    send("{\"event\":\"error\",\"msg\":\"camera niet vrijgegeven\"}");
                    return;
                }

                ctrl.build_map_stop.store(false, std::memory_order_release);

                auto stopFn = [&]() -> bool {
                    return !g_running.load(std::memory_order_acquire)
                        || ctrl.build_map_stop.load(std::memory_order_acquire);
                };

                auto statusFn = [&](const MapBuilder::ScanStatus& st) {
                    std::string msg = "{\"event\":\"map_scan_status\",";
                    msg += "\"frame\":" + std::to_string(st.frame);
                    msg += ",\"detected\":" + std::to_string(st.detected_stars);
                    msg += ",\"confirmed\":" + std::to_string(st.confirmed_stars);
                    msg += ",\"total\":" + std::to_string(st.total_stars);
                    msg += ",\"matched\":" + std::to_string(st.matched_stars);
                    msg += ",\"added\":" + std::to_string(st.new_stars);
                    const float frameW = static_cast<float>(st.frame_width);
                    const float frameH = static_cast<float>(st.frame_height);
                    msg += ",\"detections\":" + json_points_normalized(st.detected_points, frameW, frameH);
                    msg += ",\"confirmed_points\":" + json_points_normalized(st.confirmed_points, frameW, frameH);
                    msg += "}";
                    send(msg);
                };

                auto previewFn = [&](const cv::Mat& bgr) {
                    write_preview_from_bgr(&shm, bgr);
                };

                MapBuilder builder(ctrl.intr, cfg.mapper, cfg.detector,
                                   cfg.camera, ctrl.calib_height.load());
                new_map = builder.scan(stopFn, statusFn, previewFn);

                ctrl.cam_pause_req.store(false, std::memory_order_release);

                if (new_map.empty()) {
                    send("{\"event\":\"error\",\"msg\":\"scan failed\"}");
                    return;
                }

                char defName[64];
                {
                    std::time_t now_t = std::time(nullptr);
                    char ts[16]; std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S",
                                               std::localtime(&now_t));
                    std::snprintf(defName, sizeof(defName),
                        "star_map_%.3fm_%s.csv", ctrl.calib_height.load(), ts);
                }
                std::string savePath = bin_dir + "/star_maps/" + defName;
                fs::create_directories(bin_dir + "/star_maps");
                StarMap3D map3d = util::starMap3DFromLegacy(new_map);
                try {
                    StarMap3DMetadata meta3d;
                    meta3d.scale_status = ScaleStatus::Metric;
                    util::saveStarMap3D(map3d, savePath, &meta3d);
                } catch (...) {}
                ctrl.track_state.store(static_cast<int>(TrackState::Idle));
                map.star_map   = new_map;
                map.star_map_name = defName;
                map.star_index = std::make_unique<StarIndex>(
                                   StarIndex::build(map.star_map));
                map.star_map3d = map3d;
                map.star_index3d = std::make_unique<StarIndex3D>(
                    StarIndex3D::build(map.star_map3d));
                map.map3d_ready.store(true, std::memory_order_release);
                map.generation.fetch_add(1, std::memory_order_release);
                map.map_ready.store(true, std::memory_order_release);
                shm.block()->calibration_complete.store(1,
                    std::memory_order_relaxed);
                ctrl.track_state.store(static_cast<int>(TrackState::Localise));
                persist_current_map_choice(defName, cfg, ctrl);
                char buf[256];
                std::snprintf(buf, sizeof(buf),
                    "{\"event\":\"calibration_done\","
                    "\"file\":\"%s\",\"stars\":%zu}",
                    defName, map.star_map3d.size());
                send(buf);
                send(current_star_map_json(map));
            }
        }).detach();

    } else if (cmd == "stop_build_map") {
        ctrl.build_map_stop.store(true, std::memory_order_release);

    } else if (cmd == "scale_anchor") {
        // Apply a two-point scale anchor: user provides two marker IDs and
        // the physical distance between them.  Scales all map points so that
        // the map distance matches the physical distance.
        //
        // Protocol: {"cmd":"scale_anchor","id_a":0,"id_b":5,"distance_m":1.245}
        // Response: {"event":"scale_anchor_done","scale":..., "stars":...}
        //           {"event":"error","msg":"..."}
        int   id_a       = json_int(raw, "id_a", -1);
        int   id_b       = json_int(raw, "id_b", -1);
        float distance_m = static_cast<float>(json_dbl(raw, "distance_m", -1.0));

        if (id_a < 0 || id_b < 0 || id_a == id_b || distance_m <= 0.f) {
            send("{\"event\":\"error\","
                 "\"msg\":\"scale_anchor: ongeldige parameters "
                 "(id_a, id_b moeten verschillend zijn; distance_m > 0)\"}");
        } else if (!map.map_ready.load()) {
            send("{\"event\":\"error\","
                 "\"msg\":\"scale_anchor: geen sterkaart geladen\"}");
        } else {
            StarMap map_snap;
            std::string map_name;
            {
                // Snapshot under no lock (map.star_map is main-thread only for writes)
                map_snap  = map.star_map;
                map_name  = map.star_map_name;
            }

            int n = static_cast<int>(map_snap.size());
            if (id_a >= n || id_b >= n) {
                char err[128];
                std::snprintf(err, sizeof(err),
                    "{\"event\":\"error\","
                    "\"msg\":\"scale_anchor: id %d of %d buiten kaartgrootte %d\"}",
                    id_a, id_b, n);
                send(err);
            } else {
                float dx = map_snap[id_b].x - map_snap[id_a].x;
                float dy = map_snap[id_b].y - map_snap[id_a].y;
                float map_dist = std::sqrt(dx * dx + dy * dy);

                if (map_dist < 1e-6f) {
                    send("{\"event\":\"error\","
                         "\"msg\":\"scale_anchor: markers liggen te dicht bij elkaar\"}");
                } else {
                    float scale = distance_m / map_dist;

                    for (auto& p : map_snap) {
                        p.x *= scale;
                        p.y *= scale;
                    }

                    // Save scaled map (overwrite existing file)
                    std::string savePath = bin_dir + "/star_maps/" + map_name;
                    try {
                        StarMap3DMetadata meta3d;
                        meta3d.scale_status = ScaleStatus::Metric;
                        meta3d.anchors.push_back({id_a, id_b, distance_m});
                        auto map3d = util::starMap3DFromLegacy(map_snap);
                        util::saveStarMap3D(map3d, savePath, &meta3d);
                    } catch (const std::exception& e) {
                        std::fprintf(stderr, "[scale_anchor] save failed: %s\n",
                                     e.what());
                    }

                    // Update runtime state
                    ctrl.track_state.store(static_cast<int>(TrackState::Idle));
                    map.star_map   = map_snap;
                    map.star_index = std::make_unique<StarIndex>(
                                       StarIndex::build(map.star_map));
                    // Load 3D metric map into PnP pipeline
                    try {
                        auto map3d_rt = util::starMap3DFromLegacy(map_snap);
                        map.star_map3d = map3d_rt;
                        map.star_index3d = std::make_unique<StarIndex3D>(
                            StarIndex3D::build(map3d_rt));
                        map.map3d_ready.store(true, std::memory_order_release);
                    } catch (...) {}
                    map.generation.fetch_add(1, std::memory_order_release);
                    ctrl.calib_height.store(
                        ctrl.calib_height.load() * scale);
                    persist_current_map_choice(map_name, cfg, ctrl);

                    char buf[256];
                    std::snprintf(buf, sizeof(buf),
                        "{\"event\":\"scale_anchor_done\","
                        "\"scale\":%.6f,"
                        "\"stars\":%d,"
                        "\"id_a\":%d,\"id_b\":%d,"
                        "\"distance_m\":%.4f}",
                        scale, n, id_a, id_b, distance_m);
                    send(buf);
                    send(current_star_map_json(map));
                }
            }
        }

    } else if (cmd == "anchor_preview") {
        if (!map.map3d_ready.load(std::memory_order_acquire)) {
            install_star_map3d_from_legacy(map.star_map, map);
        }
        if (!map.map3d_ready.load(std::memory_order_acquire)) {
            send("{\"event\":\"error\",\"msg\":\"anchor_preview: geen sterkaart geladen\"}");
        } else {
            cv::Mat frame;
            std::vector<cv::Point2f> dets;
            uint64_t frame_ts = now_us();
            bool got_frame = false;

            auto read_latest_slot = [&]() -> bool {
                int best = -1;
                uint64_t best_seq = 0;
                for (int i = 0; i < 2; ++i) {
                    uint64_t s = (*p_slots)[i].seq.load(std::memory_order_acquire);
                    if ((s & 1) == 0 && s > best_seq) {
                        best_seq = s;
                        best = i;
                    }
                }
                if (best < 0 || best_seq == 0) return false;

                uint64_t seq1, seq2;
                do {
                    seq1 = (*p_slots)[best].seq.load(std::memory_order_acquire);
                    if (seq1 & 1) { std::this_thread::yield(); continue; }
                    frame    = (*p_slots)[best].frame.clone();
                    dets     = (*p_slots)[best].undistorted;
                    frame_ts = (*p_slots)[best].timestamp_us;
                    seq2 = (*p_slots)[best].seq.load(std::memory_order_acquire);
                } while (seq1 != seq2);
                return !frame.empty();
            };

            const bool workers_active =
                ctrl.track_state.load(std::memory_order_acquire) != static_cast<int>(TrackState::Idle);

            if (workers_active) {
                // CameraWorker is running — wait up to 3 s for a slot frame.
                // (After build_map the worker just woke up and needs ~800 ms warmup.)
                auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{3};
                while (!got_frame && std::chrono::steady_clock::now() < deadline) {
                    got_frame = read_latest_slot();
                    if (!got_frame)
                        std::this_thread::sleep_for(std::chrono::milliseconds{100});
                }
            }

            if (!got_frame && !workers_active) {
                // Camera is free (Idle state) — open directly.
                try {
                    CameraReader cam(cfg.camera);
                    std::this_thread::sleep_for(std::chrono::milliseconds{800});
                    cv::Mat raw;
                    for (int i = 0; i < 5; ++i) {
                        raw = cam.nextFrame(std::chrono::milliseconds{2000});
                        if (!raw.empty()) break;
                    }
                    if (!raw.empty()) {
                        cv::Mat remap1, remap2;
                        cv::initUndistortRectifyMap(
                            ctrl.intr.K, ctrl.intr.dist,
                            cv::noArray(), ctrl.intr.K,
                            raw.size(), CV_32FC1,
                            remap1, remap2);
                        cv::remap(raw, frame, remap1, remap2, cv::INTER_LINEAR);

                        StarDetector detector(cfg.detector);
                        dets = detector.detectRawCentroids(frame);
                        frame_ts = now_us();
                        got_frame = true;
                    }
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "[anchor_preview] direct capture failed: %s\n", e.what());
                }
            }

            if (!got_frame || frame.empty()) {
                send("{\"event\":\"anchor_preview_done\",\"ok\":false,\"matched\":0}");
            } else {
                CameraIntrinsics ci;
                ci.K    = ctrl.intr.K.clone();
                ci.dist = cv::Mat::zeros(1, 4, CV_64F);
                ci.resolution = frame.size();
                Localiser3D tmp_loc(ci, *map.star_index3d, cfg.localiser);

                auto result = tmp_loc.localise(dets, frame_ts);

                cv::Mat annotated = frame.clone();
                for (const auto& p : dets) {
                    cv::circle(annotated, p, 8, cv::Scalar(0, 200, 255), 1);
                }

                int matched = 0;
                bool overlay_ok = false;
                const StarMap3D& map3d = map.star_index3d->map3d();

                auto draw_label = [&](const cv::Point2f& pf, int id) {
                    cv::Point2i pt(static_cast<int>(pf.x), static_cast<int>(pf.y));
                    const int W = annotated.cols;
                    const int H = annotated.rows;
                    if (pt.x < 0 || pt.x >= W || pt.y < 0 || pt.y >= H)
                        return;
                    cv::circle(annotated, pt, 18, cv::Scalar(0, 255, 0), 3);
                    std::string label = std::to_string(id);
                    int baseline = 0;
                    constexpr double kFontScale = 1.0;
                    constexpr int kThickness = 3;
                    cv::Size ts = cv::getTextSize(label,
                        cv::FONT_HERSHEY_SIMPLEX, kFontScale, kThickness, &baseline);
                    cv::Point2i textOrg = pt + cv::Point2i(22, -8);
                    if (textOrg.x + ts.width + 8 >= W)
                        textOrg.x = std::max(4, pt.x - ts.width - 28);
                    if (textOrg.y - ts.height - 8 < 0)
                        textOrg.y = std::min(H - 6, pt.y + ts.height + 28);
                    cv::rectangle(annotated,
                        textOrg + cv::Point2i(-4, -ts.height - 6),
                        textOrg + cv::Point2i(ts.width + 6, baseline + 6),
                        cv::Scalar(0, 0, 0), cv::FILLED);
                    cv::rectangle(annotated,
                        textOrg + cv::Point2i(-4, -ts.height - 6),
                        textOrg + cv::Point2i(ts.width + 6, baseline + 6),
                        cv::Scalar(255, 255, 255), 1);
                    cv::putText(annotated, label, textOrg,
                        cv::FONT_HERSHEY_SIMPLEX, kFontScale,
                        cv::Scalar(255, 255, 255), kThickness);
                };

                if (result.has_value()) {
                    const auto& meas = result.value();
                    matched = meas.inliers;
                    std::vector<cv::Point3f> obj_pts;
                    obj_pts.reserve(map3d.size());
                    for (const auto& m : map3d)
                        obj_pts.push_back(m.p_world_m);

                    cv::Mat rvec, tvec;
                    T_world_cam_to_rvec_tvec(meas.T_world_cam, rvec, tvec);
                    std::vector<cv::Point2f> projected;
                    cv::projectPoints(obj_pts, rvec, tvec,
                                      ci.K, ci.dist, projected);
                    for (size_t mi = 0; mi < projected.size(); ++mi)
                        draw_label(projected[mi], map3d[mi].id);
                    overlay_ok = true;
                }

                write_preview_fit_from_mat(&shm, annotated);

                char resp[128];
                std::snprintf(resp, sizeof(resp),
                    "{\"event\":\"anchor_preview_done\","
                    "\"ok\":%s,\"matched\":%d}",
                    overlay_ok ? "true" : "false",
                    matched);
                send(resp);
            }
        }

    } else if (cmd == "start_track") {
        if (!map.map_ready.load()) {
            send("{\"event\":\"error\","
                 "\"msg\":\"no star map loaded — run build_map first\"}");
            return;
        }
        {
            // Optional "imu": false disables IMU propagation (in-memory only, not persisted).
            bool imu_on = json_bool(raw, "imu", true);
            std::lock_guard<std::mutex> lk(settings.mtx);
            settings.data.imu_enabled = imu_on;
        }
        ctrl.imu_only_mode.store(false, std::memory_order_release);
        ctrl.track_state.store(static_cast<int>(TrackState::Localise));
        shm.block()->tracking_lost.store(0, std::memory_order_relaxed);
        shm.block()->tracking_active.store(0, std::memory_order_relaxed);
        ctrl.logger.open("track", log_root, map.star_map_name,
                         ctrl.calib_height.load(), ctrl.ceiling_height_m.load());
        send("{\"event\":\"status\",\"tracking_active\":true,"
             "\"imu_only_active\":false,"
             "\"calibration_complete\":true}");

    } else if (cmd == "calib_confirm") {
        ctrl.calib_confirm.store(true, std::memory_order_release);

    } else if (cmd == "stop") {
        ctrl.logger.close();
        ctrl.imu_only_mode.store(false, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lk(settings.mtx);
            settings.data.imu_enabled = true;  // reset to default
        }
        ctrl.track_state.store(static_cast<int>(TrackState::Idle));
        shm.block()->tracking_active.store(0, std::memory_order_relaxed);
        shm.block()->tracking_lost.store(0, std::memory_order_relaxed);
        send("{\"event\":\"status\",\"tracking_active\":false,"
             "\"imu_only_active\":false,"
             "\"calibration_complete\":true}");

    } else if (cmd == "set_settings") {
        std::string ip   = json_str(raw, "freed_ip");
        int         port = json_int(raw, "freed_port", 0);
        constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
        const double off_x = json_dbl(raw, "camera_offset_x_m", kNaN);
        const double off_y = json_dbl(raw, "camera_offset_y_m", kNaN);
        const double off_z = json_dbl(raw, "camera_offset_z_m", kNaN);
        RuntimeSettings snap;
        {
            std::lock_guard<std::mutex> lk(settings.mtx);
            if (!ip.empty()) settings.data.freed_ip = ip;
            if (port > 0)   settings.data.freed_port = port;
            if (!std::isnan(off_x)) settings.data.camera_offset_x_m = off_x;
            if (!std::isnan(off_y)) settings.data.camera_offset_y_m = off_y;
            if (!std::isnan(off_z)) settings.data.camera_offset_z_m = off_z;
            snap = settings.data;
        }
        save_settings(snap);
        // Core 3 detects the changed IP/port on its next iteration and calls retarget().
        char buf[384];
        std::snprintf(buf, sizeof(buf),
            "{\"event\":\"status\","
            "\"calibration_complete\":%s,"
            "\"tracking_active\":%s,"
            "\"freed_ip\":\"%s\","
            "\"freed_port\":%d,"
            "\"camera_offset_x_m\":%.6f,"
            "\"camera_offset_y_m\":%.6f,"
            "\"camera_offset_z_m\":%.6f}",
            shm.block()->calibration_complete.load() ? "true" : "false",
            shm.block()->tracking_active.load()      ? "true" : "false",
            snap.freed_ip.c_str(), snap.freed_port,
            snap.camera_offset_x_m, snap.camera_offset_y_m, snap.camera_offset_z_m);
        send(buf);

    } else if (cmd == "set_eskf") {
        // ESKF tuning — now persisted alongside FreeD/IMU settings.
        // Optional fields: absent fields leave the current value unchanged.
        // Sentinel -1.0: json_dbl returns def=-1.0 when the key is absent.
        const float pos = static_cast<float>(json_dbl(raw, "sigma_cam_pos", -1.0));
        const float att = static_cast<float>(json_dbl(raw, "sigma_cam_att", -1.0));
        const float dec = static_cast<float>(json_dbl(raw, "vel_decay_s",   -1.0));
        RuntimeSettings snap;
        {
            std::lock_guard<std::mutex> lk(settings.mtx);
            if (pos >= 0.0f) settings.data.sigma_cam_pos = pos;
            if (att >= 0.0f) settings.data.sigma_cam_att = att;
            if (dec >= 0.0f) settings.data.vel_decay_s   = dec;
            snap = settings.data;
        }
        save_settings(snap);
        char buf[192];
        std::snprintf(buf, sizeof(buf),
            "{\"event\":\"status\","
            "\"sigma_cam_pos\":%.4f,\"sigma_cam_att\":%.4f,\"vel_decay_s\":%.3f}",
            snap.sigma_cam_pos, snap.sigma_cam_att, snap.vel_decay_s);
        send(buf);

    } else if (cmd == "set_imu_debug") {
        ImuDebugSettings snap;
        {
            std::lock_guard<std::mutex> lk(imu_debug.mtx);
            auto& d = imu_debug.data;
            d.enabled      = json_bool(raw, "enabled",      d.enabled);
            d.skip_predict = json_bool(raw, "skip_predict",  d.skip_predict);
            if (auto axis = json_str(raw, "perturb_axis"); !axis.empty()) {
                if (axis == "x" || axis == "y" || axis == "z" || axis == "none")
                    d.perturb_axis = axis;
            }
            d.perturb_deg  = json_dbl(raw, "perturb_deg", d.perturb_deg);
            if (!d.enabled) {
                d.skip_predict = false;
                d.perturb_axis = "none";
                d.perturb_deg  = 0.0;
            }
            snap = d;
        }
        char buf[192];
        std::snprintf(buf, sizeof(buf),
            "{\"event\":\"status\",\"imu_debug_enabled\":%s"
            ",\"imu_debug_skip_predict\":%s"
            ",\"imu_debug_perturb_axis\":\"%s\""
            ",\"imu_debug_perturb_deg\":%.4f}",
            snap.enabled      ? "true" : "false",
            snap.skip_predict ? "true" : "false",
            snap.perturb_axis.c_str(), snap.perturb_deg);
        send(buf);

    } else if (cmd == "set_runtime_flags") {
        bool fe = json_bool(raw, "freed_enabled", true);
        bool ie = json_bool(raw, "imu_enabled",   true);
        RuntimeSettings snap;
        {
            std::lock_guard<std::mutex> lk(settings.mtx);
            settings.data.freed_enabled = fe;
            settings.data.imu_enabled   = ie;
            snap = settings.data;
        }
        save_settings(snap);
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "{\"event\":\"status\",\"freed_enabled\":%s,\"imu_enabled\":%s}",
            fe ? "true" : "false", ie ? "true" : "false");
        send(buf);

    } else if (cmd == "set_detector_debug_mode") {
        PreviewDebugMode mode = preview_debug_mode_from_string(json_str(raw, "mode"));
        ctrl.preview_debug_mode.store(static_cast<int>(mode), std::memory_order_relaxed);
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "{\"event\":\"status\",\"detector_debug_mode\":\"%s\"}",
            preview_debug_mode_to_string(mode));
        send(buf);

    } else if (cmd == "set_grid_visible") {
        bool vis = json_bool(raw, "visible", true);
        ctrl.preview_grid.store(vis, std::memory_order_relaxed);
        send(vis ? "{\"event\":\"ok\",\"grid\":true}" : "{\"event\":\"ok\",\"grid\":false}");

    } else if (cmd == "set_pipeline_mode") {
        send("{\"event\":\"status\",\"pipeline_mode\":\"pnp\"}");
    } else if (cmd == "start_imu_only") {
        {
            std::lock_guard<std::mutex> lk(settings.mtx);
            settings.data.imu_enabled = true;
        }
        ctrl.track_state.store(static_cast<int>(TrackState::Idle));
        ctrl.imu_only_mode.store(true, std::memory_order_release);
        shm.block()->tracking_active.store(0, std::memory_order_relaxed);
        shm.block()->tracking_lost.store(0, std::memory_order_relaxed);
        send("{\"event\":\"status\",\"imu_only_active\":true}");

    } else if (cmd == "stop_imu_only") {
        ctrl.imu_only_mode.store(false, std::memory_order_release);
        send("{\"event\":\"status\",\"imu_only_active\":false}");

    } else if (cmd == "shutdown_system") {
        ctrl.track_state.store(static_cast<int>(TrackState::Idle));
        shm.block()->tracking_active.store(0, std::memory_order_relaxed);
        shm.block()->tracking_lost.store(0, std::memory_order_relaxed);
        send("{\"event\":\"status\",\"msg\":\"Afsluiten...\"}");
        std::this_thread::sleep_for(std::chrono::milliseconds{300});
        ::system("sudo systemctl poweroff");

    } else if (cmd == "reboot_system") {
        ctrl.track_state.store(static_cast<int>(TrackState::Idle));
        shm.block()->tracking_active.store(0, std::memory_order_relaxed);
        shm.block()->tracking_lost.store(0, std::memory_order_relaxed);
        send("{\"event\":\"status\",\"msg\":\"Herstarten...\"}");
        std::this_thread::sleep_for(std::chrono::milliseconds{300});
        ::system("sudo systemctl reboot");

    } else if (cmd == "save_frame") {
        std::string label = json_str(raw, "label");
        if (label.empty()) label = "frame";
        for (char& c : label)
            if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-' && c != '_')
                c = '_';
        if (label.size() > 64) label.resize(64);

        // Find the slot with the highest even seq (= latest complete frame).
        cv::Mat frame;
        std::vector<cv::Point2f> dets;
        bool got_frame = false;
        {
            int best = -1;
            uint64_t best_seq = 0;
            for (int i = 0; i < 2; ++i) {
                uint64_t s = (*p_slots)[i].seq.load(std::memory_order_acquire);
                if ((s & 1) == 0 && s > best_seq) { best_seq = s; best = i; }
            }
            if (best >= 0 && best_seq > 0) {
                uint64_t seq1, seq2;
                do {
                    seq1 = (*p_slots)[best].seq.load(std::memory_order_acquire);
                    if (seq1 & 1) { std::this_thread::yield(); continue; }
                    frame = (*p_slots)[best].frame.clone();
                    dets  = (*p_slots)[best].undistorted;
                    seq2  = (*p_slots)[best].seq.load(std::memory_order_acquire);
                } while (seq1 != seq2);
                got_frame = !frame.empty();
            }
        }

        if (!got_frame) {
            send("{\"event\":\"error\","
                 "\"msg\":\"save_frame: geen frame beschikbaar (daemon idle?)\"}");
            return;
        }

        const char* home = std::getenv("HOME");
        std::string out_dir = std::string(home ? home : "/tmp")
                            + "/.startracker/captures/" + label;
        fs::create_directories(out_dir);

        // 1. raw.png — unmodified preprocessed frame from CameraWorker
        bool write_ok = cv::imwrite(out_dir + "/raw.png", frame);

        // 2. overlay.png — centroids drawn on raw
        cv::Mat overlay = frame.clone();
        for (const auto& p : dets) {
            cv::circle(overlay, p, 12, cv::Scalar(0, 200, 255), 2);
            cv::drawMarker(overlay, p, cv::Scalar(0, 255, 0),
                           cv::MARKER_CROSS, 24, 1);
        }
        write_ok = cv::imwrite(out_dir + "/overlay.png", overlay) && write_ok;
        if (!write_ok) {
            send("{\"event\":\"error\",\"msg\":\"save_frame: imwrite mislukt (schijf vol?)\"}");
            return;
        }

        // 3. detections.json
        {
            std::string det_json = "{\"points\":[";
            for (size_t i = 0; i < dets.size(); ++i) {
                char buf[48];
                std::snprintf(buf, sizeof(buf),
                    "[%.3f,%.3f]", dets[i].x, dets[i].y);
                if (i) det_json += ",";
                det_json += buf;
            }
            det_json += "]}";
            FILE* f = std::fopen((out_dir + "/detections.json").c_str(), "w");
            if (f) { std::fputs(det_json.c_str(), f); std::fclose(f); }
        }

        // 4. pose_debug.json — real pose + reprojected map stars + NN matches
        //    Written only when tracking is active and a valid pose exists.
        bool saved_pose = false;
        if (p_cam_meas && map.map3d_ready.load()) {
            CameraPoseMeasurement meas_snap;
            {
                std::lock_guard<std::mutex> lk(p_cam_meas->mtx);
                meas_snap = p_cam_meas->meas;
            }
            if (meas_snap.valid) {
                cv::Mat rvec, tvec;
                T_world_cam_to_rvec_tvec(meas_snap.T_world_cam, rvec, tvec);

                const auto& map3d = map.star_map3d;
                std::vector<cv::Point3f> obj_pts;
                std::vector<int>         map_ids;
                obj_pts.reserve(map3d.size());
                map_ids.reserve(map3d.size());
                for (const auto& m : map3d) {
                    obj_pts.push_back(m.p_world_m);
                    map_ids.push_back(m.id);
                }

                const Intrinsics& ci = ctrl.intr;
                std::vector<cv::Point2f> projected;
                cv::projectPoints(obj_pts, rvec, tvec, ci.K, ci.dist, projected);

                const float thresh_px = cfg.localiser.threshPx;
                const int   W = frame.cols, H = frame.rows;

                // Build JSON incrementally
                std::string pj;
                pj.reserve(65536);
                pj += "{";

                // pose
                char pbuf[256];
                std::snprintf(pbuf, sizeof(pbuf),
                    "\"pose\":{\"rvec\":[%.6f,%.6f,%.6f],\"tvec\":[%.6f,%.6f,%.6f],"
                    "\"reproj_px\":%.3f,\"inliers\":%d,\"detections\":%d,\"confidence\":%.4f},",
                    rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2),
                    tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
                    meas_snap.reprojection_error_px,
                    meas_snap.inliers, meas_snap.detections, meas_snap.confidence);
                pj += pbuf;

                // intrinsics (so Python can verify reprojection)
                std::snprintf(pbuf, sizeof(pbuf),
                    "\"intrinsics\":{\"fx\":%.4f,\"fy\":%.4f,\"cx\":%.4f,\"cy\":%.4f},",
                    ci.K.at<double>(0,0), ci.K.at<double>(1,1),
                    ci.K.at<double>(0,2), ci.K.at<double>(1,2));
                pj += pbuf;

                // detections (undistorted pixel coords)
                pj += "\"detections\":[";
                for (size_t i = 0; i < dets.size(); ++i) {
                    char buf[48];
                    std::snprintf(buf, sizeof(buf),
                        "%s[%.3f,%.3f]", i ? "," : "", dets[i].x, dets[i].y);
                    pj += buf;
                }
                pj += "],";

                // projections — all map stars, with in-frame flag and map id
                pj += "\"projections\":[";
                for (size_t i = 0; i < projected.size(); ++i) {
                    bool in_frame = (projected[i].x >= 0 && projected[i].x < W &&
                                     projected[i].y >= 0 && projected[i].y < H);
                    char buf[80];
                    std::snprintf(buf, sizeof(buf),
                        "%s{\"id\":%d,\"x\":%.3f,\"y\":%.3f,\"in_frame\":%s}",
                        i ? "," : "",
                        map_ids[i], projected[i].x, projected[i].y,
                        in_frame ? "true" : "false");
                    pj += buf;
                }
                pj += "],";

                // NN matches: for each detection find nearest in-frame projection within thresh_px
                pj += "\"matches\":[";
                bool first = true;
                for (size_t i = 0; i < dets.size(); ++i) {
                    float best = thresh_px;
                    int   best_j = -1;
                    for (size_t j = 0; j < projected.size(); ++j) {
                        float dx = dets[i].x - projected[j].x;
                        float dy = dets[i].y - projected[j].y;
                        float d  = std::sqrt(dx*dx + dy*dy);
                        if (d < best) { best = d; best_j = (int)j; }
                    }
                    if (best_j >= 0) {
                        char buf[64];
                        std::snprintf(buf, sizeof(buf),
                            "%s{\"det\":%zu,\"proj\":%d,\"dist_px\":%.2f}",
                            first ? "" : ",", i, best_j, best);
                        pj += buf;
                        first = false;
                    }
                }
                pj += "]}";

                FILE* f = std::fopen((out_dir + "/pose_debug.json").c_str(), "w");
                if (f) { std::fputs(pj.c_str(), f); std::fclose(f); }
                saved_pose = true;
            }
        }

        char resp[512];
        std::snprintf(resp, sizeof(resp),
            "{\"event\":\"save_frame_done\","
            "\"label\":\"%s\","
            "\"detections\":%zu,"
            "\"pose_debug\":%s,"
            "\"path\":\"%s\"}",
            label.c_str(), dets.size(),
            saved_pose ? "true" : "false",
            json_escape(out_dir).c_str());
        send(resp);

    } else {
        send("{\"event\":\"error\",\"msg\":\"unknown command\"}");
    }
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    (void)argc;
    // Disable stdout buffering so printf output reaches journald immediately.
    setvbuf(stdout, nullptr, _IONBF, 0);

    // Use sigaction without SA_RESTART so that blocking read() calls (in the
    // SocketServer runOnce loop) are interrupted by SIGTERM/SIGINT, allowing
    // the main loop to exit cleanly within milliseconds.
    struct sigaction sa{};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;  // no SA_RESTART
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGINT,  &sa, nullptr);

    // ── Shared state (owned by main, passed by pointer to workers and handle_command) ─
    FrameSlots     slots;
    CameraMeas     cam_meas;
    VisionPose     vis_pose;
    MapState       map_state;
    ControlState   ctrl;
    SharedConfig   shared_cfg;
    SharedSettings shared_settings;
    SharedImuDebug imu_debug;

    // ── Locate binary directory ───────────────────────────────────────────────
    // Use /proc/self/exe so the path is correct regardless of how argv[0] was set
    // (e.g. launched by systemd or a script with no full path in argv[0]).
    std::string bin_dir;
    std::string repo_root;  // base for validation/ logs; overridden by STARTRACKER_LOG_DIR
    {
        char exe_buf[4096] = {};
        ssize_t n = ::readlink("/proc/self/exe", exe_buf, sizeof(exe_buf) - 1);
        bin_dir = (n > 0) ? fs::path(exe_buf).parent_path().string()
                          : fs::path(argv[0]).parent_path().string();

        const char* log_dir_env = std::getenv("STARTRACKER_LOG_DIR");
        repo_root = (log_dir_env && log_dir_env[0]) ? log_dir_env : bin_dir;
    }

    // ── Load config (config.json → CLI args) ──────────────────────────────────
    {
        std::string cfgPath = bin_dir + "/config.json";
        ctrl.config_path = cfgPath;
        if (util::loadConfig(cfgPath, shared_cfg.cfg)) {
            ctrl.min_tracking_stars.store(std::max(3, shared_cfg.cfg.tracker.minStars), std::memory_order_relaxed);
        }
    }
    // ── Load intrinsics ───────────────────────────────────────────────────────
    if (shared_cfg.cfg.intrinsics.activePath.empty()) {
        std::fprintf(stderr,
            "[FOUT] Geen intrinsics pad. Zet \"intrinsics\" in config.json\n");
        return 1;
    }
    try {
        ctrl.intr = util::loadIntrinsics(shared_cfg.cfg.intrinsics.activePath);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[FOUT] %s\n", e.what()); return 1;
    }
    std::printf("[daemon] Intrinsics: fx=%.1f\n", ctrl.intr.fx());

    // ── Load persisted runtime settings ──────────────────────────────────────
    {
        std::lock_guard<std::mutex> lk(shared_settings.mtx);
        shared_settings.data = load_settings();
    }
    std::printf("[daemon] FreeD target: %s:%d\n",
                shared_settings.data.freed_ip.c_str(), shared_settings.data.freed_port);
    // Sync ESKF defaults from AppConfig on fresh install (settings.json newly created)
    {
        std::lock_guard<std::mutex> lk(shared_settings.mtx);
        if (shared_settings.data.sigma_cam_pos == RuntimeSettings{}.sigma_cam_pos)
            shared_settings.data.sigma_cam_pos = shared_cfg.cfg.eskfSigmaCamPos;
        if (shared_settings.data.sigma_cam_att == RuntimeSettings{}.sigma_cam_att)
            shared_settings.data.sigma_cam_att = shared_cfg.cfg.eskfSigmaCamAtt;
        if (shared_settings.data.vel_decay_s == RuntimeSettings{}.vel_decay_s)
            shared_settings.data.vel_decay_s = shared_cfg.cfg.eskfVelDecayS;
        shared_settings.data.smooth_min_cutoff = shared_cfg.cfg.smoothMinCutoff;
        shared_settings.data.smooth_beta       = shared_cfg.cfg.smoothBeta;
    }

    // ── Open shared memory ────────────────────────────────────────────────────
    SharedMemoryServer shm;
    if (!shm.open()) {
        std::perror("[FOUT] SharedMemoryServer::open");
        return 1;
    }

    // ── Reset stale shared memory flags ────────────────────────────────────────────
    // The shm file is reused without truncation on restart; zero status flags
    // so the frontend does not see stale "tracking active" or "calib done" state.
    shm.block()->tracking_active.store(0, std::memory_order_relaxed);
    shm.block()->calibration_complete.store(0, std::memory_order_relaxed);
    shm.block()->tracking_lost.store(0, std::memory_order_relaxed);
    shm.block()->imu_ok.store(0, std::memory_order_relaxed);
    shm.block()->freed_connected.store(0, std::memory_order_relaxed);

    // ── Auto-load star map from config if a path is specified ─────────────────
    // This lets Core 2 start tracking immediately without requiring the user
    // to re-select the star map through the frontend after each daemon restart.
    if (!shared_cfg.cfg.starMapPath.empty()) {
        std::string mapPath = shared_cfg.cfg.starMapPath;
        if (mapPath[0] != '/') mapPath = bin_dir + "/" + mapPath;
        try {
            StarMap boot_map = util::loadStarMap(mapPath);
            map_state.star_map   = boot_map;
            map_state.star_map_name = fs::path(mapPath).filename().string();
            map_state.star_index = std::make_unique<StarIndex>(StarIndex::build(map_state.star_map));
            ctrl.calib_height.store(static_cast<float>(shared_cfg.cfg.calibHeight));
            ctrl.ceiling_height_m.store(shared_cfg.cfg.ceilingHeightM);
            map_state.generation.fetch_add(1, std::memory_order_release);
            map_state.map_ready.store(true, std::memory_order_release);
            shm.block()->calibration_complete.store(1, std::memory_order_relaxed);
            bool loaded3d = false;
            // Try companion _metric.csv for PnP pipeline
            std::string base3d = mapPath.substr(0, mapPath.rfind('.'));
            std::string path3d = base3d + "_metric.csv";
            if (fs::exists(path3d)) {
                try {
                    auto map3d = util::loadStarMap3D(path3d);
                    if (!map3d.empty()) {
                        map_state.star_map3d = map3d;
                        map_state.star_index3d = std::make_unique<StarIndex3D>(
                            StarIndex3D::build(map3d));
                        map_state.map3d_ready.store(true, std::memory_order_release);
                        loaded3d = true;
                        std::printf("[daemon] StarMap3D auto-loaded: %zu markers\n",
                                    map3d.size());
                    }
                } catch (const std::exception& e) {
                    std::fprintf(stderr,
                        "[daemon] Warning: auto-load StarMap3D failed: %s\n",
                        e.what());
                }
            }
            if (!loaded3d) {
                install_star_map3d_from_legacy(map_state.star_map, map_state);
                std::printf("[daemon] StarMap3D fallback from legacy map: %zu markers\n",
                            map_state.star_map3d.size());
            }

            std::printf("[daemon] Star map auto-loaded: %s (%zu stars, height=%.3fm)\n",
                        mapPath.c_str(), boot_map.size(), shared_cfg.cfg.calibHeight);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[daemon] Warning: auto-load star map failed: %s\n", e.what());
        }
    }

    // ── Bind socket early (frontend may connect before calibration) ───────────
    SocketServer srv(SOCK_PATH);

    // ── Command dispatch ──────────────────────────────────────────────────────
    srv.onCommand([&](const std::string& cmd, const std::string& raw,
                      SocketServer::Sender send) {
        handle_command(cmd, raw, std::move(send), &shm, &bin_dir, &repo_root,
                       &shared_cfg, &shared_settings, &imu_debug, &map_state, &ctrl, &slots,
                       &cam_meas);
    });


    if (!srv.bind()) {
        std::fprintf(stderr, "[FOUT] SocketServer::bind failed\n");
        shm.close();
        return 1;
    }
    std::printf("[daemon] Socket: %s\n", SOCK_PATH);

    // ── Launch worker threads ─────────────────────────────────────────────────
    CameraWorker   cam_worker  (&slots, &ctrl, &shared_cfg, &shm, &vis_pose);
    std::thread t1([&]{ cam_worker.run(); });
    TrackingWorker track_worker(&slots, &cam_meas, &vis_pose,
                                &map_state, &ctrl, &shared_cfg, &shm);
    std::thread t2([&]{ track_worker.run(); });
    IMUWorker      imu_worker  (&cam_meas, &vis_pose,
                                &ctrl, &shared_cfg, &shared_settings, &imu_debug, &shm);
    std::thread t3([&]{ imu_worker.run(); });

    std::printf("[daemon] Worker threads started. Waiting for frontend connection.\n");

    // ── Socket event loop (main thread) ───────────────────────────────────────
    while (g_running) {
        srv.runOnce(1000);  // blocks up to 1s waiting for a client connection
    }

    // ── Shutdown ──────────────────────────────────────────────────────────────
    std::printf("[daemon] Shutting down...\n");
    t1.join();
    t2.join();
    t3.join();
    srv.cleanup();
    shm.close();
    std::printf("[daemon] Stopped.\n");
    return 0;
}
