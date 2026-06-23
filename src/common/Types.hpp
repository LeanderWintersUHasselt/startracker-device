#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// ── Sterkaart: legacy 2D (backward compatible) ───────────────────────────────
using StarMap = std::vector<cv::Point2f>;

// ── Sterkaart: metrisch 3D (nieuw) ───────────────────────────────────────────
// p_world_m: positie in het world-frame (X,Y meters in plafondvlak, Z=0).
struct MapMarker {
    int           id = -1;
    cv::Point3f   p_world_m{0.f, 0.f, 0.f};
};
using StarMap3D = std::vector<MapMarker>;

enum class ScaleStatus { Legacy, Metric };

// Optioneel: schaalanker-paar (twee markers met bekende afstand).
struct ScaleAnchor {
    int    id_a       = -1;
    int    id_b       = -1;
    float  distance_m = 0.f;
};

struct StarMap3DMetadata {
    ScaleStatus              scale_status = ScaleStatus::Legacy;
    std::vector<ScaleAnchor> anchors;
};

// ── Camera intrinsics ─────────────────────────────────────────────────────────
struct Intrinsics {
    cv::Mat K;     // 3×3 CV_64F  (cameramatrix)
    cv::Mat dist;  // 1×N CV_64F  (vervormingscoëfficiënten)

    float fx() const { return static_cast<float>(K.at<double>(0, 0)); }
    float fy() const { return static_cast<float>(K.at<double>(1, 1)); }
    float cx() const { return static_cast<float>(K.at<double>(0, 2)); }
    float cy() const { return static_cast<float>(K.at<double>(1, 2)); }
};

// ── Resultaat van één localisatie- of trackingstap ───────────────────────────
struct PoseResult {
    float x_m        = 0.f;   // camerapositie in kaartcoördinaten (meter)
    float y_m        = 0.f;
    float theta_deg  = 0.f;   // legacy — use yaw_deg for new code
    float yaw_deg    = 0.f;   // rotation about z-axis (replaces theta_deg for new code)
    float z_m        = 0.f;   // height above ceiling (set from calibration/SVD, not fixed)
    float roll_deg   = 0.f;   // rotation about x-axis (from homography decomp in Localiser, or IMU)
    float pitch_deg  = 0.f;   // rotation about y-axis (from homography decomp in Localiser, or IMU)
    float scale_est  = 0.f;   // geschatte schaal (px/m)

    int   n_inliers    = 0;
    int   n_detections = 0;
    float match_pct    = 0.f;  // n_inliers / n_detections × 100
    float reproj_err_m = 0.f;  // gemiddelde reprojectiefout in meter

    float pose_confidence          = 0.f;   // 0.0 (lost) … 1.0 (full confidence); filled by Tracker
    bool  relocalisation_requested = false; // set true when Tracker wants Localiser to re-run

    enum class Verdict { Correct, Probable, Partial, Doubtful };
    Verdict verdict = Verdict::Doubtful;
    bool    valid   = false;

    static const char* verdictStr(Verdict v) {
        switch (v) {
            case Verdict::Correct:  return "CORRECT";
            case Verdict::Probable: return "WAARSCHIJNLIJK";
            case Verdict::Partial:  return "DEELGEBIED";
            default:                return "TWIJFELACHTIG";
        }
    }
    const char* verdictStr() const { return verdictStr(verdict); }
};
