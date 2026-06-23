#pragma once
#include "common/Config.hpp"
#include "common/Types.hpp"
#include <vector>

// Snelle IR-sticker detector.
// BGR → grijs → downsample → blur → Otsu → contour filter
//
// Distortion space contract:
//   detectRawCentroids         → pixel coords in the INPUT image (no correction)
//   detectUndistortedCentroids → detectRawCentroids + cv::undistortPoints(K, dist)
//   detect (legacy)            → alias for detectUndistortedCentroids

class StarDetectorLight {
public:
    explicit StarDetectorLight(DetectorLightConfig cfg = {});

    // Returns centroids in the same pixel space as the input image.
    // No distortion correction. Use this when:
    //   - the input is already undistorted (e.g. after cv::remap), OR
    //   - you will pass raw centroids + K + dist to solvePnP yourself.
    std::vector<cv::Point2f> detectRawCentroids(const cv::Mat& bgr) const;

    // detectRawCentroids + cv::undistortPoints(K, dist).
    // Use on a raw (distorted) image when working in undistorted pixel space
    // (e.g. legacy homography pipeline).
    std::vector<cv::Point2f> detectUndistortedCentroids(const cv::Mat& bgr,
                                                         const cv::Mat& K,
                                                         const cv::Mat& dist) const;

    // Legacy alias — calls detectUndistortedCentroids(bgr, intr.K, intr.dist).
    std::vector<cv::Point2f> detect(const cv::Mat& bgr,
                                    const Intrinsics& intr) const;

    // Compacte debug-grid (3 kolommen × 2 rijen): Raw | Gray | Downsample | Blur | Threshold | Overlay
    cv::Mat makeDebugGrid(const cv::Mat& bgr,
                          const std::vector<cv::Point2f>& detections,
                          int tileW = 960, int tileH = 540) const;

    // Full fitted ellipses from the last detectRawCentroids call,
    // in the same full-resolution coordinate space as the returned centroids.
    // Parallel to the returned vector (same order, same count).
    const std::vector<cv::RotatedRect>& lastEllipses() const { return ellipses_; }

    DetectorLightConfig& config() { return cfg_; }

private:
    DetectorLightConfig cfg_;

    struct ContourResult {
        std::vector<cv::Point> cnt_fr;  // contour points in gray_ (full-res) pixel space
        enum class Verdict { Accepted, RejectedArea, RejectedCirc } verdict;
    };

    mutable cv::Mat gray_, small_, blurred_, thresh_, accepted_;
    mutable std::vector<cv::RotatedRect> ellipses_;
    mutable std::vector<ContourResult> contour_results_;
    mutable int    n_total_contours_ = 0;
    mutable int    n_rejected_area_  = 0;
    mutable int    n_rejected_circ_  = 0;
    mutable double last_otsu_val_    = 0.0;
};
