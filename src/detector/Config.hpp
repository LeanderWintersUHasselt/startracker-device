#pragma once
#include <string>

// ════════════════════════════════════════════════════════════════════════════
//  Config.hpp — centrale parameter-hub voor het startracker-systeem
// ════════════════════════════════════════════════════════════════════════════

// ── Camera ───────────────────────────────────────────────────────────────────
struct CameraConfig {
    int   width        = 2304;
    int   height       = 1296;
    int   camera       = 0;
    int   fps          = 20;
    int   shutter      = 500;
    float gain         = 0.0f;
    float lensPosition = 10.0f;
    float awbGainR     = 0.0f;
    float awbGainB     = 0.0f;
};

// ── Sterdetectie (volledig, Python-equivalent) ────────────────────────────────
struct DetectorConfig {
    int   satMax      = 120;
    int   valMin      = 140;
    int   bgKernel    = 31;
    int   peakFloor   = 8;
    int   peakCeil    = 60;  // max Otsu threshold — prevents bright light sources from pushing Otsu too high; 255 = disabled
    int   morphKernel = 3;
    int   areaMin     = 200;
    int   areaMax     = 1000;
    float circMin     = 0.60f;
};

// ── Sterdetectie Light (snel, ~40fps) ─────────────────────────────────────────
// Grijs → downsample → threshold → contour-filter. Geen HSV, geen grote morph.
struct DetectorLightConfig {
    int   downsample   = 1;     // 1=full, 2=half, 4=quarter resolutie
    int   blurKernel   = 3;     // kleine Gaussische blur (0=geen, altijd oneven); 3 stabieliseert randpixels
    int   threshold    = 150;   // 0=Otsu adaptief, anders vaste waarde (bijv. 150)
    int   areaMin      = 200;   // min markeroppervlak in full-res px² (ds-onafhankelijk)
    int   areaMax      = 1100;  // max markeroppervlak in full-res px² (ds-onafhankelijk)
    float circMin      = 0.60f; // min circulariteit
    int   morphClose   = 0;     // morfologische closing kernelgrootte na drempel (0=uit, altijd oneven)
};

// ── Localisatie ───────────────────────────────────────────────────────────────
struct LocaliserConfig {
    int   ransacIter  = 250;
    float threshPx    = 5.0f;
    bool  useHash     = true;
    bool  verbose     = false;
};

// ── Tracker ───────────────────────────────────────────────────────────────────
struct TrackerConfig {
    float maxPx       = 50.0f;
    float maxRotDeg   = 10.0f;
    float minMatchPct = 30.0f;
    float maxReprojM  = 0.01f;  // relocalisation trigger: max inlier reprojection error in metres
    int   minStars    = 5;      // minimum detections required before tracking can be trusted
};

// ── Kaartbouw ─────────────────────────────────────────────────────────────────
struct MapBuilderConfig {
    int   mapFps         = 3;
    float mergeRadiusPx = 37.0f;
    int   minFrameCount  = 5;
    int   maxNewPerFrame = 5;
    float minInlierRatio = 0.6f;
};

// ── Intrinsics-bestanden ─────────────────────────────────────────────────────
struct IntrinsicsPaths {
    std::string activePath = "";
    std::string backupPath = "";
    std::string fixedPath  = "";
};

// ── Applicatie ────────────────────────────────────────────────────────────────
struct AppConfig {
    DetectorConfig      detector{};
    DetectorLightConfig detectorLight{};
    LocaliserConfig     localiser{};
    TrackerConfig       tracker{};
    CameraConfig        camera{};
    MapBuilderConfig    mapper{};

    float        calibHeight    = 1.351f;
    std::string  starMapPath    = "";
    IntrinsicsPaths intrinsics{};
    float        eskfNoiseScale = 1.0f;   // ESKF process noise tuning multiplier
    float        eskfSigmaImuAtt   = 0.02f;   // BNO085 attitude sigma [rad] (~1.1°)
    float        eskfSigmaCamPos   = 0.05f;   // camera position sigma [m]
    // Startup-only (require restart); not exposed in config_json / UI panel.
    // Runtime velocity decay is tunable without restart via set_eskf command.
    int          eskfMinInitInliers  = 4;      // minimum inliers to accept init pose
    float        eskfMinInitReprojPx = 5.0f;   // max reprojection error [px] to accept init pose
    float        eskfVelDecayS       = 0.0f;   // default decay; 0=off; >0=velocity e-fold decay [s]
    float        ceilingHeightM       = 2.92f;  // floor-to-ceiling height [m]; runtime-settable via set_config
    float        eskfSigmaCamAtt      = 0.005f; // camera attitude measurement sigma [rad]; tighter than BNO085
    bool         useHeavyDetector     = false;  // use full StarDetector (slower, better centroids) during tracking
    float        smoothMinCutoff      = 0.0f;   // One Euro Filter min cutoff [Hz]; 0 = off
    float        smoothBeta           = 0.5f;   // One Euro Filter speed coefficient
};
