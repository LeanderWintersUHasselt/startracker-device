#include "util/ConfigLoader.hpp"
#include <fstream>
#include <sstream>
#include <cstdio>

namespace util {

// ── Minimale JSON-helpers ─────────────────────────────────────────────────────

static bool   jBool (const std::string& j, const std::string& k, bool   def);
static int    jInt  (const std::string& j, const std::string& k, int    def);
static float  jFloat(const std::string& j, const std::string& k, float  def);
static std::string jStr(const std::string& j, const std::string& k, const std::string& def);

static std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) return {};
    return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}

// Zoek "key": waarde in de JSON-string. Werkt voor geneste objecten niet perfect,
// maar onze config is een vlakke structuur per sectie.
static std::string findValue(const std::string& json, const std::string& key) {
    auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return {};
    pos = json.find(':', pos);
    if (pos == std::string::npos) return {};
    ++pos;
    while (pos < json.size() && std::isspace(json[pos])) ++pos;
    // lees tot komma, } of newline
    size_t end = pos;
    while (end < json.size() && json[end] != ',' && json[end] != '}' && json[end] != '\n') ++end;
    std::string val = json.substr(pos, end - pos);
    // strip spaties en aanhalingstekens
    while (!val.empty() && (std::isspace(val.back())  || val.back()  == '"')) val.pop_back();
    while (!val.empty() && (std::isspace(val.front()) || val.front() == '"')) val.erase(0,1);
    return val;
}

static int   jInt  (const std::string& j, const std::string& k, int   def) {
    auto v = findValue(j, k); return v.empty() ? def : std::stoi(v); }
static float jFloat(const std::string& j, const std::string& k, float def) {
    auto v = findValue(j, k); return v.empty() ? def : std::stof(v); }
static bool  jBool (const std::string& j, const std::string& k, bool  def) {
    auto v = findValue(j, k);
    if (v.empty()) return def;
    return v == "true" || v == "1"; }
static std::string jStr(const std::string& j, const std::string& k, const std::string& def) {
    auto v = findValue(j, k); return v.empty() ? def : v; }

// ── loadConfig ────────────────────────────────────────────────────────────────

bool loadConfig(const std::string& path, AppConfig& cfg) {
    std::string json = readFile(path);
    if (json.empty()) return false;

    // Camera
    cfg.camera.width        = jInt  (json, "width",        cfg.camera.width);
    cfg.camera.height       = jInt  (json, "height",       cfg.camera.height);
    cfg.camera.camera       = jInt  (json, "camera",       cfg.camera.camera);
    cfg.camera.fps          = jInt  (json, "fps",          cfg.camera.fps);
    cfg.camera.shutter      = jInt  (json, "shutter",      cfg.camera.shutter);
    cfg.camera.gain         = jFloat(json, "gain",         cfg.camera.gain);
    cfg.camera.lensPosition = jFloat(json, "lens_position",cfg.camera.lensPosition);
    cfg.camera.awbGainR     = jFloat(json, "awb_gain_r",   cfg.camera.awbGainR);
    cfg.camera.awbGainB     = jFloat(json, "awb_gain_b",   cfg.camera.awbGainB);

    // Detector
    cfg.detector.satMax      = jInt  (json, "sat_max",      cfg.detector.satMax);
    cfg.detector.valMin      = jInt  (json, "val_min",      cfg.detector.valMin);
    cfg.detector.bgKernel    = jInt  (json, "bg_kernel",    cfg.detector.bgKernel);
    cfg.detector.peakFloor   = jInt  (json, "peak_floor",   cfg.detector.peakFloor);
    cfg.detector.peakCeil    = jInt  (json, "peak_ceil",    cfg.detector.peakCeil);
    cfg.detector.morphKernel = jInt  (json, "morph_kernel", cfg.detector.morphKernel);
    cfg.detector.areaMin     = jInt  (json, "area_min",     cfg.detector.areaMin);
    cfg.detector.areaMax     = jInt  (json, "area_max",     cfg.detector.areaMax);
    cfg.detector.circMin     = jFloat(json, "circ_min",     cfg.detector.circMin);

    // DetectorLight
    cfg.detectorLight.downsample  = jInt  (json, "light_downsample",  cfg.detectorLight.downsample);
    cfg.detectorLight.blurKernel  = jInt  (json, "light_blur_kernel", cfg.detectorLight.blurKernel);
    cfg.detectorLight.threshold   = jInt  (json, "light_threshold",   cfg.detectorLight.threshold);
    cfg.detectorLight.areaMin     = jInt  (json, "light_area_min",    cfg.detectorLight.areaMin);
    cfg.detectorLight.areaMax     = jInt  (json, "light_area_max",    cfg.detectorLight.areaMax);
    cfg.detectorLight.circMin     = jFloat(json, "light_circ_min",    cfg.detectorLight.circMin);
    cfg.detectorLight.morphClose  = jInt  (json, "light_morph_close", cfg.detectorLight.morphClose);

    // Localiser
    cfg.localiser.ransacIter = jInt  (json, "ransac_iter",  cfg.localiser.ransacIter);
    cfg.localiser.threshPx   = jFloat(json, "thresh_px",    cfg.localiser.threshPx);
    cfg.localiser.verbose    = jBool (json, "verbose",      cfg.localiser.verbose);

    // Tracker
    cfg.tracker.maxPx        = jFloat(json, "max_px",       cfg.tracker.maxPx);
    cfg.tracker.maxRotDeg    = jFloat(json, "max_rot_deg",  cfg.tracker.maxRotDeg);
    cfg.tracker.minMatchPct  = jFloat(json, "min_match_pct",cfg.tracker.minMatchPct);
    cfg.tracker.maxReprojM   = jFloat(json, "max_reproj_m", cfg.tracker.maxReprojM);
    cfg.tracker.minStars     = jInt  (json, "min_tracking_stars", cfg.tracker.minStars);
    if (cfg.tracker.minStars < 3) cfg.tracker.minStars = 3;

    // MapBuilder
    cfg.mapper.mapFps         = jInt  (json, "map_fps",        cfg.mapper.mapFps);
    cfg.mapper.mergeRadiusPx  = jFloat(json, "merge_radius_px",cfg.mapper.mergeRadiusPx);
    cfg.mapper.minFrameCount  = jInt  (json, "min_frame_count",cfg.mapper.minFrameCount);
    cfg.mapper.maxNewPerFrame = jInt  (json, "max_new_per_frame",cfg.mapper.maxNewPerFrame);
    cfg.mapper.minInlierRatio = jFloat(json, "min_inlier_ratio",cfg.mapper.minInlierRatio);

    // App-niveau
    cfg.calibHeight              = jFloat(json, "calib_height",      cfg.calibHeight);
    cfg.starMapPath              = jStr  (json, "star_map",          cfg.starMapPath);
    cfg.intrinsics.activePath    = jStr  (json, "intrinsics",        cfg.intrinsics.activePath);
    cfg.intrinsics.backupPath    = jStr  (json, "intrinsics_backup", cfg.intrinsics.backupPath);
    cfg.intrinsics.fixedPath     = jStr  (json, "intrinsics_fixed",  cfg.intrinsics.fixedPath);
    cfg.eskfNoiseScale = jFloat(json, "eskf_noise_scale", cfg.eskfNoiseScale);
    cfg.eskfSigmaImuAtt = jFloat(json, "eskf_sigma_imu_att", cfg.eskfSigmaImuAtt);
    cfg.eskfSigmaCamPos = jFloat(json, "eskf_sigma_cam_pos", cfg.eskfSigmaCamPos);
    cfg.eskfMinInitInliers  = jInt  (json, "eskf_min_init_inliers",   cfg.eskfMinInitInliers);
    cfg.eskfMinInitReprojPx = jFloat(json, "eskf_min_init_reproj_px", cfg.eskfMinInitReprojPx);
    cfg.eskfVelDecayS       = jFloat(json, "eskf_vel_decay_s",        cfg.eskfVelDecayS);
    cfg.ceilingHeightM      = jFloat(json, "ceiling_height_m",        cfg.ceilingHeightM);
    cfg.eskfSigmaCamAtt     = jFloat(json, "eskf_sigma_cam_att", cfg.eskfSigmaCamAtt);
    cfg.useHeavyDetector    = jBool (json, "use_heavy_detector", cfg.useHeavyDetector);

    return true;
}

// ── saveConfig ────────────────────────────────────────────────────────────────

void saveConfig(const std::string& path, const AppConfig& cfg) {
    const auto& c = cfg.camera;
    const auto& d = cfg.detector;
    const auto& dl= cfg.detectorLight;
    const auto& l = cfg.localiser;
    const auto& t = cfg.tracker;
    const auto& m = cfg.mapper;

    // Bouw eerst de string op, schrijf dan in één keer (voorkomt leeg bestand bij crash)
    char buf[4096];
    int n = std::snprintf(buf, sizeof(buf),
        "{\n"
        "  \"width\":         %d,\n"
        "  \"height\":        %d,\n"
        "  \"camera\":        %d,\n"
        "  \"fps\":           %d,\n"
        "  \"shutter\":       %d,\n"
        "  \"gain\":          %.2f,\n"
        "  \"lens_position\": %.1f,\n"
        "  \"awb_gain_r\":    %.2f,\n"
        "  \"awb_gain_b\":    %.2f,\n\n"
        "  \"sat_max\":       %d,\n"
        "  \"val_min\":       %d,\n"
        "  \"bg_kernel\":     %d,\n"
        "  \"peak_floor\":    %d,\n"
        "  \"peak_ceil\":     %d,\n"
        "  \"morph_kernel\":  %d,\n"
        "  \"area_min\":      %d,\n"
        "  \"area_max\":      %d,\n"
        "  \"circ_min\":      %.2f,\n\n"
        "  \"light_downsample\":  %d,\n"
        "  \"light_blur_kernel\": %d,\n"
        "  \"light_threshold\":   %d,\n"
        "  \"light_area_min\":    %d,\n"
        "  \"light_area_max\":    %d,\n"
        "  \"light_circ_min\":    %.2f,\n"
        "  \"light_morph_close\": %d,\n\n"
        "  \"ransac_iter\":   %d,\n"
        "  \"thresh_px\":     %.1f,\n"
        "  \"verbose\":       %s,\n\n"
        "  \"max_px\":        %.1f,\n"
        "  \"max_rot_deg\":   %.1f,\n"
        "  \"min_match_pct\": %.1f,\n"
        "  \"max_reproj_m\":  %.4f,\n"
        "  \"min_tracking_stars\": %d,\n\n"
        "  \"map_fps\":           %d,\n"
        "  \"merge_radius_px\":   %.1f,\n"
        "  \"min_frame_count\":   %d,\n"
        "  \"max_new_per_frame\": %d,\n"
        "  \"min_inlier_ratio\":  %.2f,\n\n"
        "  \"calib_height\":         %.4f,\n"
        "  \"star_map\":             \"%s\",\n"
        "  \"intrinsics\":           \"%s\",\n"
        "  \"intrinsics_backup\":    \"%s\",\n"
        "  \"intrinsics_fixed\":     \"%s\",\n"
        "  \"eskf_noise_scale\":     %.2f,\n"
        "  \"eskf_sigma_imu_att\":   %.4f,\n"
        "  \"eskf_sigma_cam_pos\":   %.4f,\n"
        "  \"eskf_min_init_inliers\":  %d,\n"
        "  \"eskf_min_init_reproj_px\": %.1f,\n"
        "  \"eskf_vel_decay_s\":       %.1f,\n"
        "  \"ceiling_height_m\":       %.2f,\n"
        "  \"eskf_sigma_cam_att\":     %.4f,\n"
        "  \"use_heavy_detector\":    %s\n"
        "}\n",
        c.width, c.height, c.camera, c.fps, c.shutter, c.gain, c.lensPosition,
        c.awbGainR, c.awbGainB,
        d.satMax, d.valMin, d.bgKernel, d.peakFloor, d.peakCeil, d.morphKernel,
        d.areaMin, d.areaMax, d.circMin,
        dl.downsample, dl.blurKernel, dl.threshold,
        dl.areaMin, dl.areaMax, dl.circMin, dl.morphClose,
        l.ransacIter, l.threshPx, l.verbose ? "true" : "false",
        t.maxPx, t.maxRotDeg, t.minMatchPct, t.maxReprojM, t.minStars,
        m.mapFps, m.mergeRadiusPx, m.minFrameCount,
        m.maxNewPerFrame, m.minInlierRatio,
        cfg.calibHeight,
        cfg.starMapPath.c_str(),
        cfg.intrinsics.activePath.c_str(),
        cfg.intrinsics.backupPath.c_str(),
        cfg.intrinsics.fixedPath.c_str(),
        cfg.eskfNoiseScale,
        cfg.eskfSigmaImuAtt,
        cfg.eskfSigmaCamPos,
        cfg.eskfMinInitInliers,
        cfg.eskfMinInitReprojPx,
        cfg.eskfVelDecayS,
        cfg.ceilingHeightM,
        cfg.eskfSigmaCamAtt,
        cfg.useHeavyDetector ? "true" : "false");

    if (n <= 0 || n >= static_cast<int>(sizeof(buf))) {
        std::fprintf(stderr, "[FOUT] saveConfig: buffer te klein\n");
        return;
    }

    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) { std::fprintf(stderr, "[FOUT] saveConfig: kan niet schrijven: %s\n", path.c_str()); return; }
    std::fwrite(buf, 1, n, f);
    std::fclose(f);
}


} // namespace util
