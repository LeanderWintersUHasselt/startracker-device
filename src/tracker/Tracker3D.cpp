#include "tracker/Tracker3D.hpp"
#include <cmath>
#include <cstdio>
#include <limits>

Tracker3D::Tracker3D(CameraIntrinsics intr, TrackerConfig cfg)
    : intr_(std::move(intr)), cfg_(cfg) {}

std::optional<Tracker3D::Result> Tracker3D::track(
        const CameraPoseMeasurement& prevPose,
        const std::vector<cv::Point2f>& detPoints,
        const StarMap3D& map3d,
        uint64_t timestamp_us) const {

    if (!prevPose.valid || detPoints.empty() || map3d.empty())
        return std::nullopt;

    // ── 1. Project all map markers to pixel space using previous pose ────────────
    cv::Mat rvec, tvec;
    T_world_cam_to_rvec_tvec(prevPose.T_world_cam, rvec, tvec);

    std::vector<cv::Point3f> objAll;
    objAll.reserve(map3d.size());
    for (const auto& m : map3d)
        objAll.push_back(m.p_world_m);

    std::vector<cv::Point2f> projAll;
    cv::projectPoints(objAll, rvec, tvec, intr_.K, intr_.dist, projAll);

    // ── 2. Local NN matching: for each projection find nearest detection ─────────
    const float maxPx2 = cfg_.maxPx * cfg_.maxPx;
    int nd = static_cast<int>(detPoints.size());
    int nm = static_cast<int>(map3d.size());

    std::vector<cv::Point3f> objMatched;
    std::vector<cv::Point2f> imgMatched;
    objMatched.reserve(nm);
    imgMatched.reserve(nm);

    // Track which detections are already used to avoid double-assignment
    std::vector<bool> usedDet(nd, false);

    for (int mi = 0; mi < nm; ++mi) {
        const cv::Point2f& proj = projAll[mi];

        // Skip markers projected outside a generous image border
        if (proj.x < -50 || proj.x > intr_.resolution.width  + 50 ||
            proj.y < -50 || proj.y > intr_.resolution.height + 50)
            continue;

        float bestD2 = maxPx2;
        int   bestDi = -1;
        for (int di = 0; di < nd; ++di) {
            if (usedDet[di]) continue;
            float dx = detPoints[di].x - proj.x;
            float dy = detPoints[di].y - proj.y;
            float d2 = dx*dx + dy*dy;
            if (d2 < bestD2) { bestD2 = d2; bestDi = di; }
        }

        if (bestDi >= 0) {
            objMatched.push_back(objAll[mi]);
            imgMatched.push_back(detPoints[bestDi]);
            usedDet[bestDi] = true;
        }
    }

    int minStars = std::max(3, cfg_.minStars);
    if (static_cast<int>(objMatched.size()) < minStars) {
        // Too few matches → request relocalisation
        Tracker3D::Result r;
        r.relocalise_requested = true;
        return r;
    }

    // ── 3. refinePose ────────────────────────────────────────────────────────────
    auto meas = PoseEstimator::refinePose(
        prevPose.T_world_cam,
        objMatched, imgMatched,
        intr_.K, intr_.dist,
        timestamp_us);

    // ── 4. Reprojection gate ─────────────────────────────────────────────────────
    if (!meas.valid || meas.reprojection_error_px > cfg_.maxReprojM * intr_.fx()) {
        Tracker3D::Result r;
        r.relocalise_requested = true;
        return r;
    }

    // ── 5. Rotation consistency gate ─────────────────────────────────────────────
    // Catch mirror-solution flips that pass the reprojection check.
    // A genuine camera motion of >maxRotDeg per frame is physically impossible
    // at 20 fps; a valid solution must be within that bound of the previous pose.
    {
        const cv::Matx33d& R_prev = prevPose.T_world_cam.R;
        const cv::Matx33d& R_new  = meas.T_world_cam.R;
        cv::Matx33d dR = R_prev.t() * R_new;
        double cosA = std::max(-1.0, std::min(1.0,
                        (dR(0,0) + dR(1,1) + dR(2,2) - 1.0) * 0.5));
        if (std::acos(cosA) * (180.0 / CV_PI) > cfg_.maxRotDeg) {
            Tracker3D::Result r;
            r.relocalise_requested = true;
            return r;
        }
    }

    Tracker3D::Result r;
    r.meas = meas;
    r.relocalise_requested = false;
    return r;
}
