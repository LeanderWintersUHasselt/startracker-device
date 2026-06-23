#include "localiser/Localiser3D.hpp"
#include <cstdio>

Localiser3D::Localiser3D(CameraIntrinsics intr,
                          const StarIndex3D& index,
                          LocaliserConfig cfg)
    : intr_(std::move(intr)), index_(index), cfg_(cfg) {}

std::optional<CameraPoseMeasurement> Localiser3D::localise(
        const std::vector<cv::Point2f>& detPoints,
        uint64_t timestamp_us) const {

    if (detPoints.size() < 4 || index_.size() < 4)
        return std::nullopt;

    // ── 1. Get candidate (detection_idx, map_marker_idx) pairs ──────────────────
    auto candidates = index_.query(detPoints);

    // ── 2. Brute-force fallback: if StarIndex3D gives too few candidates,
    //       use all (det, marker) combinations and let RANSAC handle it ──────────
    const StarMap3D& map3d = index_.map3d();

    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;
    objPts.reserve(candidates.size());
    imgPts.reserve(candidates.size());

    if (candidates.size() >= 4) {
        for (auto& [di, mi] : candidates) {
            imgPts.push_back(detPoints[di]);
            objPts.push_back(map3d[mi].p_world_m);
        }
    } else {
        // Brute-force: all (det, marker) pairs
        int nd = static_cast<int>(detPoints.size());
        int nm = static_cast<int>(map3d.size());
        objPts.reserve(nd * nm);
        imgPts.reserve(nd * nm);
        for (int di = 0; di < nd; ++di) {
            for (int mi = 0; mi < nm; ++mi) {
                imgPts.push_back(detPoints[di]);
                objPts.push_back(map3d[mi].p_world_m);
            }
        }
    }

    if (objPts.size() < 4) return std::nullopt;

    if (cfg_.verbose)
        std::printf("[LOC3D] det=%d  candidates=%d (brute=%s)\n",
                    (int)detPoints.size(), (int)objPts.size(),
                    candidates.size() < 4 ? "yes" : "no");

    // ── 3. solvePnPRansac ────────────────────────────────────────────────────────
    auto meas = PoseEstimator::estimatePoseRansac(
        objPts, imgPts,
        intr_.K, intr_.dist,
        timestamp_us,
        cfg_.ransacIter,
        cfg_.threshPx);

    if (!meas.valid) return std::nullopt;

    // ── 4. Map-bounds plausibility check ────────────────────────────────────────
    const cv::Vec3d& t = meas.T_world_cam.t;
    float x = static_cast<float>(t[0]);
    float y = static_cast<float>(t[1]);

    float min_x = map3d[0].p_world_m.x, max_x = min_x;
    float min_y = map3d[0].p_world_m.y, max_y = min_y;
    for (const auto& m : map3d) {
        min_x = std::min(min_x, m.p_world_m.x);
        max_x = std::max(max_x, m.p_world_m.x);
        min_y = std::min(min_y, m.p_world_m.y);
        max_y = std::max(max_y, m.p_world_m.y);
    }
    constexpr float kMarginM = 1.0f;
    if (x < min_x - kMarginM || x > max_x + kMarginM ||
        y < min_y - kMarginM || y > max_y + kMarginM) {
        if (cfg_.verbose)
            std::printf("[LOC3D] pose out of map bounds (%.2f, %.2f)\n", x, y);
        return std::nullopt;
    }

    if (cfg_.verbose)
        std::printf("[LOC3D] OK: inliers=%d/%d reproj=%.2fpx\n",
                    meas.inliers, meas.detections, meas.reprojection_error_px);

    return meas;
}
