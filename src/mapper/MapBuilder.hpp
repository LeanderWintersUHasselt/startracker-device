#pragma once
#include "common/Types.hpp"
#include "common/Config.hpp"
#include "detector/StarDetector.hpp"
#include <functional>
#include <string>

class MapBuilder {
public:
    MapBuilder(Intrinsics intr, MapBuilderConfig cfg, DetectorConfig detCfg,
               CameraConfig camCfg, float calibHeight);

    struct CalibResult {
        bool  ok             = false;
        float height_m       = 0.f;
        float scale_px_per_m = 0.f;
        float t_norm         = 0.f;
        float n_z            = 0.f;
        int   inliers        = 0;
        int   total_matches  = 0;
        float calib_dist_m   = 0.f;
    };

    struct ScanStatus {
        int frame = 0;
        int total_stars = 0;
        int confirmed_stars = 0;
        int detected_stars = 0;
        int matched_stars = 0;
        int new_stars = 0;
        int frame_width = 0;
        int frame_height = 0;
        std::vector<cv::Point2f> detected_points;
        std::vector<cv::Point2f> confirmed_points;
    };

    CalibResult calibrateHeight(
        float calibDistanceM = 0.10f,
        std::function<bool()>                   waitFn  = nullptr,
        std::function<void(const std::string&)> sendFn  = nullptr);

    StarMap scan(std::function<bool()> stopFn = nullptr,
                 std::function<void(const ScanStatus&)> statusFn = nullptr,
                 std::function<void(const cv::Mat&)> previewFn = nullptr);

private:
    struct Star {
        cv::Point2f pos;
        int         count;
    };

    std::tuple<int,int,int> processFrame(
        const std::vector<cv::Point2f>& detUndist);

    static std::vector<std::pair<int,int>> mutualNN(
        const std::vector<cv::Point2f>& a,
        const std::vector<cv::Point2f>& b,
        float maxDist);

    Intrinsics       intr_;
    MapBuilderConfig cfg_;
    DetectorConfig   detCfg_;
    CameraConfig     camCfg_;
    float            calibHeight_;
    float            scaleHint_;

    std::vector<Star> stars_;
    cv::Mat           H_est_ref_to_curr_;
    bool              initialized_ = false;
};
