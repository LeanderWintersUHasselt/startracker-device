#pragma once
#include "common/Config.hpp"
#include "common/Types.hpp"
#include "geometry/Calibration.hpp"
#include "geometry/Transform.hpp"
#include "pose/PoseEstimator.hpp"
#include <optional>
#include <vector>

// Per-frame tracker using projection + local NN matching + solvePnP.
//
// Algorithm per frame:
//   1. Project all StarMap3D markers to raw pixel space using T_world_cam
//   2. For each projection: find nearest raw detection within maxPx
//   3. Run refinePose (IPPE + LM) on correspondences
//   4. Reprojection-error gate: reject if too high
//
// Falls back to re-localisation request if too few matches or error too high.

class Tracker3D {
public:
    Tracker3D(CameraIntrinsics intr, TrackerConfig cfg = {});

    struct Result {
        CameraPoseMeasurement meas;
        bool relocalise_requested = false;
    };

    // prevPose:   T_world_cam from previous frame (used to project markers)
    // detPoints:  raw pixel detections (same distortion space as intr)
    // map3d:      metric star map
    std::optional<Result> track(
        const CameraPoseMeasurement& prevPose,
        const std::vector<cv::Point2f>& detPoints,
        const StarMap3D& map3d,
        uint64_t timestamp_us) const;

private:
    CameraIntrinsics intr_;
    TrackerConfig    cfg_;
};
