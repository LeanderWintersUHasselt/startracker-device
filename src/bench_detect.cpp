/*
 * bench_detect — offline detector benchmark on saved frames.
 *
 * Loads all PNG/JPG files from <frames_dir>, runs each detector config,
 * reports mean detection time, count, and centroid stability (frame-to-frame
 * std of each matched star's pixel position).
 *
 * Usage: bench_detect <frames_dir> [--config <path>] [--intrinsics <path>]
 */

#include "common/Config.hpp"
#include "detector/StarDetector.hpp"
#include "detector/StarDetectorLight.hpp"
#include "util/ConfigLoader.hpp"
#include "util/Intrinsics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace fs = std::filesystem;
using clk = std::chrono::steady_clock;

static double ms_since(clk::time_point t0) {
    return std::chrono::duration<double, std::milli>(clk::now() - t0).count();
}

// Match centroids across frames using nearest-neighbour to frame 0's detections.
// Tolerates occasional extra/missing detections. Returns mean per-star std in x/y,
// or -1 if fewer than 2 frames have detections or no star appears in all frames.
static std::pair<double,double> centroid_stability(
        const std::vector<std::vector<cv::Point2f>>& per_frame,
        int /*img_w*/, int /*img_h*/) {
    if (per_frame.size() < 2) return {-1, -1};

    // Use frame 0 as the reference set of star positions.
    const auto& ref = per_frame[0];
    int n_ref = static_cast<int>(ref.size());
    if (n_ref == 0) return {-1, -1};

    // For each reference star, collect matched positions across all frames.
    constexpr float kMaxDist = 20.f;   // px — beyond this it's a different star/FP
    std::vector<std::vector<cv::Point2f>> tracks(n_ref);
    for (int i = 0; i < n_ref; ++i) tracks[i].push_back(ref[i]);

    for (size_t fi = 1; fi < per_frame.size(); ++fi) {
        const auto& frame = per_frame[fi];
        for (int i = 0; i < n_ref; ++i) {
            float best_d = kMaxDist * kMaxDist;
            const cv::Point2f* best = nullptr;
            for (const auto& pt : frame) {
                auto d = pt - ref[i];
                float d2 = d.x*d.x + d.y*d.y;
                if (d2 < best_d) { best_d = d2; best = &pt; }
            }
            if (best) tracks[i].push_back(*best);
        }
    }

    double sum_sx = 0, sum_sy = 0;
    int count = 0;
    for (const auto& track : tracks) {
        if (track.size() < 2) continue;
        double mean_x = 0, mean_y = 0;
        for (const auto& p : track) { mean_x += p.x; mean_y += p.y; }
        mean_x /= track.size(); mean_y /= track.size();
        double var_x = 0, var_y = 0;
        for (const auto& p : track) {
            var_x += (p.x - mean_x) * (p.x - mean_x);
            var_y += (p.y - mean_y) * (p.y - mean_y);
        }
        var_x /= track.size(); var_y /= track.size();
        sum_sx += std::sqrt(var_x);
        sum_sy += std::sqrt(var_y);
        ++count;
    }
    if (count == 0) return {-1, -1};
    return { sum_sx / count, sum_sy / count };
}

struct BenchResult {
    std::string name;
    double mean_ms{0}, min_ms{0}, max_ms{0};
    double mean_dets{0};
    double stab_x{-1}, stab_y{-1};
};

// ── Shared helpers for inline pipeline variants ───────────────────────────────

// BGR → V channel + white mask (S ≤ satMax AND V ≥ valMin)
static void hsv_white_mask(const cv::Mat& bgr, const DetectorConfig& cfg,
                            cv::Mat& V_out, cv::Mat& white_mask_out) {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> ch(3);
    cv::split(hsv, ch);
    cv::Mat chroma, val;
    cv::threshold(ch[1], chroma, cfg.satMax, 255, cv::THRESH_BINARY_INV);
    cv::threshold(ch[2], val,    cfg.valMin, 255, cv::THRESH_BINARY);
    cv::bitwise_and(chroma, val, white_mask_out);
    V_out = ch[2].clone();
}

// Otsu + peakFloor threshold → contour filter → centroids
static std::vector<cv::Point2f> threshold_filter(const cv::Mat& enhanced,
                                                   const cv::Mat& white_mask,
                                                   const DetectorConfig& cfg) {
    cv::Mat tmp;
    double otsuVal = cv::threshold(enhanced, tmp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    double peakThr = std::max(static_cast<double>(cfg.peakFloor), otsuVal);
    cv::Mat thresh, mask_final;
    cv::threshold(enhanced, thresh, peakThr, 255, cv::THRESH_BINARY);
    cv::bitwise_and(thresh, white_mask, mask_final);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_final, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point2f> centroids;
    for (const auto& cnt : contours) {
        float area = static_cast<float>(cv::contourArea(cnt));
        if (area < cfg.areaMin || area > cfg.areaMax) continue;
        float perim = static_cast<float>(cv::arcLength(cnt, true));
        if (perim < 1e-3f) continue;
        float circ = 4.f * static_cast<float>(CV_PI) * area / (perim * perim);
        if (circ < cfg.circMin) continue;
        cv::Moments m = cv::moments(cnt);
        if (m.m00 < 1e-6) continue;
        centroids.emplace_back(static_cast<float>(m.m10 / m.m00),
                                static_cast<float>(m.m01 / m.m00));
    }
    return centroids;
}

// Full heavy pipeline → enhanced, gray, mask_final (DS2+OPEN + Otsu + white mask)
static void heavy_pipeline(const cv::Mat& bgr, const DetectorConfig& cfg,
                            cv::Mat& enhanced_out, cv::Mat& gray_out, cv::Mat& mask_out) {
    cv::cvtColor(bgr, gray_out, cv::COLOR_BGR2GRAY);
    cv::Mat V, wm;
    hsv_white_mask(bgr, cfg, V, wm);
    cv::Mat V_ds;
    cv::resize(V, V_ds, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    int bgK_half = std::max(3, cfg.bgKernel / 2);
    if (bgK_half % 2 == 0) ++bgK_half;
    cv::Mat bgKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {bgK_half, bgK_half});
    cv::Mat bg_ds, bg;
    cv::morphologyEx(V_ds, bg_ds, cv::MORPH_OPEN, bgKernel);
    cv::resize(bg_ds, bg, V.size(), 0, 0, cv::INTER_LINEAR);
    cv::subtract(V, bg, enhanced_out);
    cv::Mat tmp;
    double otsuVal = cv::threshold(enhanced_out, tmp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    double peakThr = std::max(static_cast<double>(cfg.peakFloor), otsuVal);
    cv::Mat thresh;
    cv::threshold(enhanced_out, thresh, peakThr, 255, cv::THRESH_BINARY);
    cv::bitwise_and(thresh, wm, mask_out);
}

// Contour filter + intensity-weighted centroid sampled from sample_img
static std::vector<cv::Point2f> intensity_weighted_filter(
        const cv::Mat& sample_img, const cv::Mat& mask_final, const DetectorConfig& cfg) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_final, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point2f> centroids;
    for (const auto& cnt : contours) {
        float area = static_cast<float>(cv::contourArea(cnt));
        if (area < cfg.areaMin || area > cfg.areaMax) continue;
        float perim = static_cast<float>(cv::arcLength(cnt, true));
        if (perim < 1e-3f) continue;
        float circ = 4.f * static_cast<float>(CV_PI) * area / (perim * perim);
        if (circ < cfg.circMin) continue;
        cv::Rect bbox = cv::boundingRect(cnt)
                      & cv::Rect(0, 0, sample_img.cols, sample_img.rows);
        if (bbox.empty()) continue;
        cv::Mat blob_mask = cv::Mat::zeros(bbox.size(), CV_8U);
        std::vector<cv::Point> cnt_s;
        cnt_s.reserve(cnt.size());
        for (const auto& p : cnt)
            cnt_s.push_back({p.x - bbox.x, p.y - bbox.y});
        cv::fillPoly(blob_mask, std::vector<std::vector<cv::Point>>{cnt_s}, 255);
        cv::Mat roi = cv::Mat::zeros(bbox.size(), CV_8U);
        sample_img(bbox).copyTo(roi, blob_mask);
        cv::Moments m = cv::moments(roi, false);
        if (m.m00 < 1e-6) continue;
        centroids.emplace_back(
            static_cast<float>(m.m10 / m.m00) + bbox.x,
            static_cast<float>(m.m01 / m.m00) + bbox.y);
    }
    return centroids;
}

// ── Benchmark runners ─────────────────────────────────────────────────────────

static BenchResult run_light(const std::string& name,
                              const std::vector<cv::Mat>& frames,
                              const DetectorLightConfig& dc,
                              const std::string& grid_path) {
    StarDetectorLight det(dc);
    std::vector<double> times;
    std::vector<std::vector<cv::Point2f>> per_frame;
    cv::Mat last_grid;

    for (const auto& bgr : frames) {
        auto t0 = clk::now();
        auto dets = det.detectRawCentroids(bgr);
        times.push_back(ms_since(t0));
        per_frame.push_back(dets);
        if (&bgr == &frames.back())
            last_grid = det.makeDebugGrid(bgr, dets, 960, 540);
    }

    if (!last_grid.empty() && !grid_path.empty())
        cv::imwrite(grid_path, last_grid);

    auto [sx, sy] = centroid_stability(per_frame, frames[0].cols, frames[0].rows);

    BenchResult r;
    r.name      = name;
    r.mean_ms   = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    r.min_ms    = *std::min_element(times.begin(), times.end());
    r.max_ms    = *std::max_element(times.begin(), times.end());
    r.mean_dets = std::accumulate(per_frame.begin(), per_frame.end(), 0.0,
                    [](double a, const auto& v){ return a + v.size(); }) / per_frame.size();
    r.stab_x    = sx;
    r.stab_y    = sy;
    return r;
}

static BenchResult run_heavy(const std::string& name,
                              const std::vector<cv::Mat>& frames,
                              const DetectorConfig& dc,
                              const std::string& grid_path) {
    StarDetector det(dc);
    std::vector<double> times;
    std::vector<std::vector<cv::Point2f>> per_frame;
    cv::Mat last_grid;

    for (const auto& bgr : frames) {
        auto t0 = clk::now();
        auto dets = det.detectRawCentroids(bgr);
        times.push_back(ms_since(t0));
        per_frame.push_back(dets);
        if (&bgr == &frames.back())
            last_grid = det.makeDebugGrid(bgr, dets, 960, 540);
    }

    if (!last_grid.empty() && !grid_path.empty())
        cv::imwrite(grid_path, last_grid);

    auto [sx, sy] = centroid_stability(per_frame, frames[0].cols, frames[0].rows);

    BenchResult r;
    r.name      = name;
    r.mean_ms   = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    r.min_ms    = *std::min_element(times.begin(), times.end());
    r.max_ms    = *std::max_element(times.begin(), times.end());
    r.mean_dets = std::accumulate(per_frame.begin(), per_frame.end(), 0.0,
                    [](double a, const auto& v){ return a + v.size(); }) / per_frame.size();
    r.stab_x    = sx;
    r.stab_y    = sy;
    return r;
}

// Inline pipeline variant — fn is the full detect function
static BenchResult run_inline(const std::string& name,
                               const std::vector<cv::Mat>& frames,
                               std::function<std::vector<cv::Point2f>(const cv::Mat&)> fn) {
    std::vector<double> times;
    std::vector<std::vector<cv::Point2f>> per_frame;

    for (const auto& bgr : frames) {
        auto t0 = clk::now();
        auto dets = fn(bgr);
        times.push_back(ms_since(t0));
        per_frame.push_back(dets);
    }

    auto [sx, sy] = centroid_stability(per_frame, frames[0].cols, frames[0].rows);

    BenchResult r;
    r.name      = name;
    r.mean_ms   = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    r.min_ms    = *std::min_element(times.begin(), times.end());
    r.max_ms    = *std::max_element(times.begin(), times.end());
    r.mean_dets = std::accumulate(per_frame.begin(), per_frame.end(), 0.0,
                    [](double a, const auto& v){ return a + v.size(); }) / per_frame.size();
    r.stab_x    = sx;
    r.stab_y    = sy;
    return r;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: bench_detect <frames_dir> [--config <path>] [--intrinsics <path>]\n");
        return 1;
    }

    std::string frames_dir = argv[1];
    std::string cfg_path, intr_path;

    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--config"     && i+1 < argc) cfg_path  = argv[++i];
        else if (a == "--intrinsics" && i+1 < argc) intr_path = argv[++i];
    }

    AppConfig cfg;
    if (cfg_path.empty()) {
        std::string bin_dir = fs::path(argv[0]).parent_path().string();
        cfg_path = bin_dir + "/config.json";
    }
    util::loadConfig(cfg_path, cfg);
    if (!intr_path.empty()) cfg.intrinsics.activePath = intr_path;

    // Load frames
    std::vector<std::pair<std::string,cv::Mat>> entries;
    for (auto& entry : fs::directory_iterator(frames_dir)) {
        auto ext = entry.path().extension().string();
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
            cv::Mat img = cv::imread(entry.path().string());
            if (!img.empty()) entries.push_back({entry.path().filename().string(), img});
        }
    }
    std::sort(entries.begin(), entries.end(),
              [](const auto& a, const auto& b){ return a.first < b.first; });

    std::vector<cv::Mat> frames;
    for (auto& [name, img] : entries) {
        frames.push_back(img);
        std::printf("Loaded: %s\n", name.c_str());
    }
    if (frames.empty()) { std::fprintf(stderr, "No frames in %s\n", frames_dir.c_str()); return 1; }
    std::printf("Loaded %zu frames (%dx%d)\n\n", frames.size(), frames[0].cols, frames[0].rows);

    std::vector<BenchResult> results;

    // ── Light detector variants ───────────────────────────────────────────────
    // Baseline label built from the actual config values so it always matches.
    auto light_label = [](const DetectorLightConfig& c, const char* tag = "") -> std::string {
        char buf[128];
        std::string thr = c.threshold == 0 ? "Otsu" : std::to_string(c.threshold);
        std::snprintf(buf, sizeof(buf), "LIGHT  blur=%d thr=%s morph=%d%s",
                      c.blurKernel, thr.c_str(), c.morphClose, tag);
        return buf;
    };
    auto heavy_label = [](const DetectorConfig& c, const char* tag = "") -> std::string {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "HEAVY  OPEN k=%d full-res%s", c.bgKernel, tag);
        return buf;
    };

    results.push_back(run_light(light_label(cfg.detectorLight, "  ← installed"), frames,
        cfg.detectorLight, "/tmp/bench_light_baseline.jpg"));

    { auto c = cfg.detectorLight; c.blurKernel = 0; c.threshold = 0; c.morphClose = 0;
      results.push_back(run_light(light_label(c), frames, c, "/tmp/bench_light_blur0_otsu_morph0.jpg")); }

    { auto c = cfg.detectorLight; c.blurKernel = 3; c.threshold = 0; c.morphClose = 0;
      results.push_back(run_light(light_label(c), frames, c, "/tmp/bench_light_blur3_otsu_morph0.jpg")); }

    { auto c = cfg.detectorLight; c.blurKernel = 5; c.threshold = 0; c.morphClose = 0;
      results.push_back(run_light(light_label(c), frames, c, "/tmp/bench_light_blur5_otsu_morph0.jpg")); }

    { auto c = cfg.detectorLight; c.blurKernel = 3; c.threshold = 100; c.morphClose = 0;
      results.push_back(run_light(light_label(c), frames, c, "/tmp/bench_light_blur3_thr100.jpg")); }

    { auto c = cfg.detectorLight; c.blurKernel = 3; c.threshold = 150; c.morphClose = 0;
      results.push_back(run_light(light_label(c), frames, c, "/tmp/bench_light_blur3_thr150.jpg")); }

    { auto c = cfg.detectorLight; c.blurKernel = 3; c.threshold = 200; c.morphClose = 0;
      results.push_back(run_light(light_label(c), frames, c, "/tmp/bench_light_blur3_thr200.jpg")); }

    // ── Heavy detector — reference configs ────────────────────────────────────
    results.push_back(run_heavy(heavy_label(cfg.detector, "  ← installed"), frames,
        cfg.detector, "/tmp/bench_heavy_baseline.jpg"));

    { auto c = cfg.detector; c.bgKernel = 0;
      results.push_back(run_heavy("HEAVY  OPEN k=0  (no bg-sub)", frames, c, "/tmp/bench_heavy_nobg.jpg")); }

    { auto c = cfg.detector; c.bgKernel = 19;
      results.push_back(run_heavy(heavy_label(c), frames, c, "/tmp/bench_heavy_k19.jpg")); }

    { auto c = cfg.detector; c.bgKernel = 23;
      results.push_back(run_heavy(heavy_label(c), frames, c, "/tmp/bench_heavy_k23.jpg")); }

    // ── Heavy — downsample 2× before OPEN ────────────────────────────────────
    // Half-res = quarter the pixels; kernel scales to half (still > half-res star size ~8-10px).
    // Background upsampled back before subtraction.
    for (int k : {13, 15}) {
        DetectorConfig dc = cfg.detector;
        results.push_back(run_inline(
            std::string("HEAVY  DS2+OPEN k=") + std::to_string(k) + " (≈k" + std::to_string(k*2) + " full)",
            frames,
            [dc, k](const cv::Mat& bgr) {
                cv::Mat V, wm;
                hsv_white_mask(bgr, dc, V, wm);
                cv::Mat V_ds;
                cv::resize(V, V_ds, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
                int kk = (k % 2 == 0) ? k + 1 : k;
                cv::Mat bgKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {kk, kk});
                cv::Mat bg_ds;
                cv::morphologyEx(V_ds, bg_ds, cv::MORPH_OPEN, bgKernel);
                cv::Mat bg;
                cv::resize(bg_ds, bg, V.size(), 0, 0, cv::INTER_LINEAR);
                cv::Mat enhanced;
                cv::subtract(V, bg, enhanced);
                return threshold_filter(enhanced, wm, dc);
            }));
    }

    // ── Heavy — Gaussian blur as background estimate (full-res) ───────────────
    // Gaussian is separable → O(W*H*sigma) vs O(W*H*k²) for morphological OPEN.
    // sigma ≈ k/3: σ=10 ≈ k=30, σ=8 ≈ k=24, σ=6 ≈ k=18.
    // Unlike OPEN, Gaussian does NOT guarantee zero star signal in background —
    // test whether detection quality is maintained.
    for (double sigma : {10.0, 8.0, 6.0}) {
        DetectorConfig dc = cfg.detector;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "HEAVY  Gauss σ=%.0f full-res (≈k%.0f)", sigma, sigma * 3);
        results.push_back(run_inline(buf, frames,
            [dc, sigma](const cv::Mat& bgr) {
                cv::Mat V, wm;
                hsv_white_mask(bgr, dc, V, wm);
                int ks = static_cast<int>(sigma * 6) | 1;
                cv::Mat bg;
                cv::GaussianBlur(V, bg, cv::Size(ks, ks), sigma);
                cv::Mat enhanced;
                cv::subtract(V, bg, enhanced);
                return threshold_filter(enhanced, wm, dc);
            }));
    }

    // ── Heavy — downsample 2× + Gaussian ─────────────────────────────────────
    // Half-res Gaussian at half sigma = same spatial scale as full-res at full sigma.
    // Should be ~4× faster than full-res Gaussian at the same effective scale.
    for (double sigma : {8.0, 5.0}) {
        DetectorConfig dc = cfg.detector;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "HEAVY  DS2+Gauss σ=%.0f (≈σ%.0f full)", sigma, sigma * 2);
        results.push_back(run_inline(buf, frames,
            [dc, sigma](const cv::Mat& bgr) {
                cv::Mat V, wm;
                hsv_white_mask(bgr, dc, V, wm);
                cv::Mat V_ds;
                cv::resize(V, V_ds, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
                int ks = static_cast<int>(sigma * 6) | 1;
                cv::Mat bg_ds;
                cv::GaussianBlur(V_ds, bg_ds, cv::Size(ks, ks), sigma);
                cv::Mat bg;
                cv::resize(bg_ds, bg, V.size(), 0, 0, cv::INTER_LINEAR);
                cv::Mat enhanced;
                cv::subtract(V, bg, enhanced);
                return threshold_filter(enhanced, wm, dc);
            }));
    }

    // ── Centroid comparison: enhanced_ vs gray (same DS2+OPEN pipeline) ──────
    { DetectorConfig dc = cfg.detector;
      results.push_back(run_inline("HEAVY  DS2+OPEN centroid=enhanced", frames,
        [dc](const cv::Mat& bgr) {
            cv::Mat enhanced, gray, mask;
            heavy_pipeline(bgr, dc, enhanced, gray, mask);
            return intensity_weighted_filter(enhanced, mask, dc);
        })); }

    { DetectorConfig dc = cfg.detector;
      results.push_back(run_inline("HEAVY  DS2+OPEN centroid=gray", frames,
        [dc](const cv::Mat& bgr) {
            cv::Mat enhanced, gray, mask;
            heavy_pipeline(bgr, dc, enhanced, gray, mask);
            return intensity_weighted_filter(gray, mask, dc);
        })); }

    // ── Print results ─────────────────────────────────────────────────────────
    std::printf("\n%-52s  %7s  %7s  %7s  %5s  %8s  %8s\n",
                "Config", "mean_ms", "min_ms", "max_ms", "dets", "stab_x(px)", "stab_y(px)");
    std::printf("%-52s  %7s  %7s  %7s  %5s  %8s  %8s\n",
                std::string(52,'-').c_str(),
                "-------","------","------","----","----------","----------");
    for (const auto& r : results) {
        char sx[16], sy[16];
        if (r.stab_x < 0) {
            std::snprintf(sx, sizeof(sx), " unstable");
            std::snprintf(sy, sizeof(sy), " unstable");
        } else {
            std::snprintf(sx, sizeof(sx), "%8.3f", r.stab_x);
            std::snprintf(sy, sizeof(sy), "%8.3f", r.stab_y);
        }
        std::printf("%-52s  %7.1f  %7.1f  %7.1f  %5.1f  %s  %s\n",
                    r.name.c_str(), r.mean_ms, r.min_ms, r.max_ms, r.mean_dets, sx, sy);
    }
    std::printf("\nDebug grids saved to /tmp/bench_*.jpg\n");
    return 0;
}
