#pragma once
#include "common/Types.hpp"
#include "geometry/Transform.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdint>

// ── CameraPoseMeasurement ─────────────────────────────────────────────────────
// Output of every PnP solve: T_world_cam + diagnostics.

struct CameraPoseMeasurement {
    RigidTransform T_world_cam;
    uint64_t       timestamp_us         = 0;  // detection-done time
    uint64_t       capture_timestamp_us = 0;  // when nextFrame() returned (before detection)
    double         reprojection_error_px = 0.0;
    int            inliers              = 0;
    int            detections           = 0;
    double         confidence           = 0.0;  // 0..1
    bool           valid                = false;
};

// ── PoseEstimator ─────────────────────────────────────────────────────────────
// Centralises all solvePnP logic.
//
// Distortion contract:
//   imagePoints must be RAW pixel coordinates (not pre-undistorted).
//   K and dist are used internally by solvePnP and projectPoints.
//   This avoids double-undistort when combined with detectRawCentroids().

class PoseEstimator {
public:
    // Global relocalization: RANSAC over all candidate correspondences.
    // Use when prior pose is unknown (cold start / tracking loss).
    //
    // objectPoints: 3D positions of candidate markers in world frame (metres)
    // imagePoints:  corresponding raw pixel detections (same order)
    // K, dist:      camera intrinsics (CV_64F)
    static CameraPoseMeasurement estimatePoseRansac(
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints,
        const cv::Mat& K,
        const cv::Mat& dist,
        uint64_t timestamp_us,
        int ransacIterations = 1000,
        float ransacThresholdPx = 3.0f);

    // Per-frame refinement with a known initial guess.
    // Uses SOLVEPNP_IPPE (analytically optimal for coplanar Z=0 markers),
    // then refines with solvePnPRefineLM.
    // Falls back to iterative solvePnP if IPPE is ill-conditioned.
    //
    // initialGuess: T_world_cam from previous frame or ESKF prediction.
    static CameraPoseMeasurement refinePose(
        const RigidTransform& initialGuess,
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints,
        const cv::Mat& K,
        const cv::Mat& dist,
        uint64_t timestamp_us);

private:
    // Choose the physically valid IPPE solution (positive Z in camera frame).
    // Uses initialGuess rotation as tiebreaker to prevent mirror-solution flips.
    static RigidTransform selectIppeSolution(
        const std::vector<cv::Mat>& rvecs,
        const std::vector<cv::Mat>& tvecs,
        const RigidTransform& initialGuess);

    static double computeReprojError(
        const RigidTransform& T_world_cam,
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints,
        const cv::Mat& K,
        const cv::Mat& dist);

    static double computeConfidence(double reproj_px, int inliers, int detections);
};
