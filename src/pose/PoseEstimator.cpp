#include "pose/PoseEstimator.hpp"
#include <cmath>
#include <limits>

// ── estimatePoseRansac ────────────────────────────────────────────────────────

CameraPoseMeasurement PoseEstimator::estimatePoseRansac(
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints,
        const cv::Mat& K,
        const cv::Mat& dist,
        uint64_t timestamp_us,
        int ransacIterations,
        float ransacThresholdPx) {

    CameraPoseMeasurement result;
    result.timestamp_us = timestamp_us;
    result.detections   = static_cast<int>(imagePoints.size());

    if (static_cast<int>(objectPoints.size()) < 4 ||
        objectPoints.size() != imagePoints.size()) {
        return result;
    }

    cv::Mat rvec, tvec;
    std::vector<int> inliers;

    bool ok = cv::solvePnPRansac(
        objectPoints, imagePoints,
        K, dist,
        rvec, tvec,
        false,           // useExtrinsicGuess
        ransacIterations,
        ransacThresholdPx,
        0.99,            // confidence
        inliers,
        cv::SOLVEPNP_SQPNP);

    if (!ok || inliers.empty()) return result;

    // Refine inlier set with LM
    std::vector<cv::Point3f> obj_in;
    std::vector<cv::Point2f> img_in;
    for (int idx : inliers) {
        obj_in.push_back(objectPoints[idx]);
        img_in.push_back(imagePoints[idx]);
    }
    cv::solvePnPRefineLM(obj_in, img_in, K, dist, rvec, tvec);

    result.T_world_cam         = T_world_cam_from_rvec_tvec(rvec, tvec);
    result.inliers             = static_cast<int>(inliers.size());
    result.reprojection_error_px = computeReprojError(result.T_world_cam,
                                                       obj_in, img_in, K, dist);
    result.confidence          = computeConfidence(result.reprojection_error_px,
                                                    result.inliers,
                                                    result.detections);
    result.valid               = result.reprojection_error_px < 8.0 &&
                                  result.inliers >= 3;
    return result;
}

// ── refinePose ────────────────────────────────────────────────────────────────

CameraPoseMeasurement PoseEstimator::refinePose(
        const RigidTransform& initialGuess,
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints,
        const cv::Mat& K,
        const cv::Mat& dist,
        uint64_t timestamp_us) {

    CameraPoseMeasurement result;
    result.timestamp_us = timestamp_us;
    result.detections   = static_cast<int>(imagePoints.size());

    if (static_cast<int>(objectPoints.size()) < 4 ||
        objectPoints.size() != imagePoints.size()) {
        return result;
    }

    // Try IPPE first (analytically optimal for coplanar markers, Z=0)
    bool ippe_ok = false;
    RigidTransform T_ippe;

    if (objectPoints.size() >= 4) {
        std::vector<cv::Mat> rvecs, tvecs;
        std::vector<double> reproj_errors;
        try {
            cv::solvePnPGeneric(objectPoints, imagePoints, K, dist,
                                rvecs, tvecs, false,
                                cv::SOLVEPNP_IPPE,
                                cv::noArray(), cv::noArray(),
                                reproj_errors);
            if (rvecs.size() >= 1) {
                T_ippe = selectIppeSolution(rvecs, tvecs, initialGuess);
                ippe_ok = true;
            }
        } catch (...) {
            ippe_ok = false;
        }
    }

    cv::Mat rvec, tvec;
    if (ippe_ok) {
        // Convert IPPE result to rvec/tvec for LM refinement
        T_world_cam_to_rvec_tvec(T_ippe, rvec, tvec);
    } else {
        // Fall back: use initial guess + iterative solvePnP
        T_world_cam_to_rvec_tvec(initialGuess, rvec, tvec);
        cv::solvePnP(objectPoints, imagePoints, K, dist, rvec, tvec,
                     true, cv::SOLVEPNP_ITERATIVE);
    }

    // LM sub-pixel refinement
    cv::solvePnPRefineLM(objectPoints, imagePoints, K, dist, rvec, tvec);

    result.T_world_cam = T_world_cam_from_rvec_tvec(rvec, tvec);
    result.inliers     = static_cast<int>(objectPoints.size());  // all used
    result.reprojection_error_px = computeReprojError(result.T_world_cam,
                                                       objectPoints, imagePoints,
                                                       K, dist);
    result.confidence  = computeConfidence(result.reprojection_error_px,
                                            result.inliers, result.detections);
    result.valid       = result.reprojection_error_px < 8.0 && result.inliers >= 3;
    return result;
}

// ── selectIppeSolution ────────────────────────────────────────────────────────
// IPPE always returns two solutions. Prefer physically valid ones (markers in
// front of the camera: tz > 0). Among valid candidates, pick the rotation
// closest to initialGuess to prevent mirror-solution flips.

RigidTransform PoseEstimator::selectIppeSolution(
        const std::vector<cv::Mat>& rvecs,
        const std::vector<cv::Mat>& tvecs,
        const RigidTransform& initialGuess) {

    // Collect physically valid candidates (markers in front of camera).
    std::vector<size_t> candidates;
    for (size_t i = 0; i < rvecs.size(); ++i) {
        if (tvecs[i].at<double>(2) > 0.0)
            candidates.push_back(i);
    }
    if (candidates.empty()) {
        for (size_t i = 0; i < rvecs.size(); ++i)
            candidates.push_back(i);
    }

    // Pick the candidate whose rotation is closest to the initial guess.
    size_t best = candidates[0];
    double bestAngle = std::numeric_limits<double>::max();
    for (size_t i : candidates) {
        RigidTransform T = T_world_cam_from_rvec_tvec(rvecs[i], tvecs[i]);
        cv::Matx33d dR = initialGuess.R.t() * T.R;
        double cosA = std::max(-1.0, std::min(1.0, (dR(0,0) + dR(1,1) + dR(2,2) - 1.0) * 0.5));
        double angle = std::acos(cosA);
        if (angle < bestAngle) { bestAngle = angle; best = i; }
    }
    return T_world_cam_from_rvec_tvec(rvecs[best], tvecs[best]);
}

// ── computeReprojError ────────────────────────────────────────────────────────

double PoseEstimator::computeReprojError(
        const RigidTransform& T_world_cam,
        const std::vector<cv::Point3f>& objectPoints,
        const std::vector<cv::Point2f>& imagePoints,
        const cv::Mat& K,
        const cv::Mat& dist) {

    if (objectPoints.empty()) return std::numeric_limits<double>::max();

    cv::Mat rvec, tvec;
    T_world_cam_to_rvec_tvec(T_world_cam, rvec, tvec);

    std::vector<cv::Point2f> projected;
    cv::projectPoints(objectPoints, rvec, tvec, K, dist, projected);

    double sum = 0.0;
    for (size_t i = 0; i < imagePoints.size(); ++i) {
        double dx = projected[i].x - imagePoints[i].x;
        double dy = projected[i].y - imagePoints[i].y;
        sum += std::sqrt(dx * dx + dy * dy);
    }
    return sum / static_cast<double>(imagePoints.size());
}

// ── computeConfidence ─────────────────────────────────────────────────────────

double PoseEstimator::computeConfidence(double reproj_px, int inliers,
                                         int detections) {
    if (inliers < 3 || detections < 3) return 0.0;

    // Inlier ratio component
    double inlier_ratio = static_cast<double>(inliers) / detections;

    // Reprojection quality component (saturates at 1px = max confidence)
    double reproj_score = 1.0 - std::min(reproj_px / 8.0, 1.0);

    return 0.5 * inlier_ratio + 0.5 * reproj_score;
}
