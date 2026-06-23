#pragma once
#include "common/Types.hpp"
#include "common/Config.hpp"
#include <vector>

// Detecteert IR-reflecterende stickers in een BGR-frame.
// Algoritme:
//   1. BGR → HSV
//   2. White mask (laag S, hoog V)
//   3. Achtergrondaftrek via Gaussische blur op V-kanaal
//   4. Drempelstelling (Otsu-vloer) + morfologische open+close
//   5. Contour-filter (oppervlak + circulariteit)
//   6. Centroid via momenten + undistort

class StarDetector {
public:
    explicit StarDetector(DetectorConfig cfg = {});

    // Returns centroids in the same pixel space as the input image.
    // No distortion correction.
    std::vector<cv::Point2f> detectRawCentroids(const cv::Mat& bgr) const;

    // detectRawCentroids + cv::undistortPoints(K, dist).
    std::vector<cv::Point2f> detectUndistortedCentroids(const cv::Mat& bgr,
                                                         const cv::Mat& K,
                                                         const cv::Mat& dist) const;

    // Legacy alias — calls detectUndistortedCentroids(bgr, intr.K, intr.dist).
    std::vector<cv::Point2f> detect(const cv::Mat& bgr,
                                    const Intrinsics& intr) const;

    // Compacte debug-grid (3 kolommen × 2 rijen) van de laatste detect()-aanroep.
    // Kolommen: Raw | Value | Enhanced | MaskFinal | MaskClean | Overlay
    // Rij 1: Raw, Value, Enhanced   Rij 2: MaskFinal, MaskClean, Overlay
    // tileW/tileH = grootte van elke tegel (pixels).
    cv::Mat makeDebugGrid(const cv::Mat& bgr,
                          const std::vector<cv::Point2f>& detections,
                          int tileW = 640, int tileH = 360) const;

    // Sla tussenliggende stappen op als losse JPEG-bestanden.
    // Roep aan NA detect(). prefix bijv. "/tmp/calib_ref"
    void saveDebugImages(const cv::Mat& bgr,
                         const std::vector<cv::Point2f>& detections,
                         const std::string& prefix) const;

    DetectorConfig& config() { return cfg_; }  // voor live parameter-aanpassing

private:
    DetectorConfig cfg_;

    mutable cv::Mat hsv_, chroma_, val_, white_mask_, enhanced_, thresh_,
                    mask_final_, mask_clean_, accepted_;
    mutable cv::Mat bg_, kernel_;
    mutable int    n_total_contours_ = 0;
    mutable int    n_rejected_area_  = 0;
    mutable int    n_rejected_circ_  = 0;
    mutable double last_otsu_val_    = 0.0;
};
