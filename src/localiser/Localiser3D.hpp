#pragma once
#include "common/Config.hpp"
#include "common/Types.hpp"
#include "geometry/Calibration.hpp"
#include "localiser/StarIndex3D.hpp"
#include "pose/PoseEstimator.hpp"
#include <optional>
#include <vector>

// Global relocalization using StarMap3D + solvePnPRansac.
//
// Distortion contract:
//   detPoints must be in the SAME distortion space as K and dist.
//   If detPoints are from detectRawCentroids on the undistorted frame,
//   pass dist = zero (points already in undistorted space).
//   If detPoints are from detectRawCentroids on the raw frame,
//   pass the full K and dist from the calibration.

class Localiser3D {
public:
    Localiser3D(CameraIntrinsics intr,
                const StarIndex3D& index,
                LocaliserConfig cfg = {});

    // Returns nullopt if localisation fails.
    // detPoints: 2D pixel detections (distortion space must match intr).
    // timestamp_us: CLOCK_MONOTONIC microseconds of the frame.
    std::optional<CameraPoseMeasurement> localise(
        const std::vector<cv::Point2f>& detPoints,
        uint64_t timestamp_us) const;

private:
    CameraIntrinsics    intr_;
    const StarIndex3D&  index_;
    LocaliserConfig     cfg_;
};
