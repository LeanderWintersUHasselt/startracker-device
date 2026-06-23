#pragma once
// Header-only lineaire algebra helpers.
// Elke functie is een directe C++-vertaling van de overeenkomstige Python-code.

#include "common/Types.hpp"
#include <cmath>
#include <optional>
#include <vector>

namespace linalg {

// ── Gesloten-vorm 4-DOF similarity fit via least-squares ─────────────────────
// Per puntpaar (sx,sy)→(dx,dy):
//   [ sx  -sy  1  0 ] [a ]   [dx]
//   [ sy   sx  0  1 ] [b ] = [dy]
//                      [tx]
//                      [ty]
// Geeft 2×3 CV_64F matrix terug (of nullopt als n < 2).
inline std::optional<cv::Mat> fitSimilarityLS(
        const std::vector<cv::Point2f>& src,
        const std::vector<cv::Point2f>& dst) {
    int n = static_cast<int>(src.size());
    if (n < 2) return std::nullopt;

    cv::Mat M(2 * n, 4, CV_64F);
    cv::Mat rhs(2 * n, 1, CV_64F);

    for (int i = 0; i < n; ++i) {
        double sx = src[i].x, sy = src[i].y;
        double dx = dst[i].x, dy = dst[i].y;
        M.at<double>(2*i,   0) =  sx;  M.at<double>(2*i,   1) = -sy;
        M.at<double>(2*i,   2) =  1.0; M.at<double>(2*i,   3) =  0.0;
        M.at<double>(2*i+1, 0) =  sy;  M.at<double>(2*i+1, 1) =  sx;
        M.at<double>(2*i+1, 2) =  0.0; M.at<double>(2*i+1, 3) =  1.0;
        rhs.at<double>(2*i)   = dx;
        rhs.at<double>(2*i+1) = dy;
    }

    cv::Mat x;
    cv::solve(M, rhs, x, cv::DECOMP_SVD);

    double a  = x.at<double>(0), b  = x.at<double>(1);
    double tx = x.at<double>(2), ty = x.at<double>(3);
    return (cv::Mat_<double>(2, 3) << a, -b, tx, b, a, ty);
}

// ── PoseResult → A (2×3 pixel→meter) ─────────────────────────────────────────
// Inverse van extractPose: geeft de similarity matrix terug voor een gekende pose.
inline cv::Mat aFromPose(const PoseResult& pose, const Intrinsics& intr) {
    double s     = pose.scale_est;
    double theta = pose.theta_deg * CV_PI / 180.0;
    double ct    = std::cos(theta), st = std::sin(theta);
    double cx    = intr.cx(),       cy = intr.cy();

    double tx = pose.x_m - s * (ct * cx - st * cy);
    double ty = pose.y_m - s * (st * cx + ct * cy);

    return (cv::Mat_<double>(2, 3) << s*ct, -s*st, tx, s*st, s*ct, ty);
}

// ── A (2×3) → PoseResult ─────────────────────────────────────────────────────
// Converteert een gevonden similarity matrix naar een leesbaar PoseResult.
// inlierPairs: paren (det_i, map_j) die als inliers beschouwd worden.
inline PoseResult extractPose(
        const cv::Mat& A,
        const Intrinsics& intr,
        const std::vector<cv::Point2f>& detUndist,
        const std::vector<std::pair<int,int>>& inlierPairs,
        const StarMap& map) {
    PoseResult p;

    // Camerapositie = projectie van het principaal punt
    double cx = intr.cx(), cy = intr.cy();
    p.x_m  = static_cast<float>(A.at<double>(0,0)*cx + A.at<double>(0,1)*cy + A.at<double>(0,2));
    p.y_m  = static_cast<float>(A.at<double>(1,0)*cx + A.at<double>(1,1)*cy + A.at<double>(1,2));

    // Rotatie en schaal uit de 2×2 rotatie-schaal submatrix
    p.theta_deg = static_cast<float>(
        std::atan2(A.at<double>(1,0), A.at<double>(0,0)) * 180.0 / CV_PI);
    p.scale_est = static_cast<float>(
        std::sqrt(A.at<double>(0,0)*A.at<double>(0,0) +
                  A.at<double>(1,0)*A.at<double>(1,0)));

    // Statistieken
    p.n_inliers    = static_cast<int>(inlierPairs.size());
    p.n_detections = static_cast<int>(detUndist.size());
    p.match_pct    = p.n_detections > 0
                   ? 100.f * p.n_inliers / p.n_detections : 0.f;

    // Gemiddelde reprojectiefout
    double errSum = 0.0;
    for (auto& [di, mi] : inlierPairs) {
        double px = A.at<double>(0,0)*detUndist[di].x + A.at<double>(0,1)*detUndist[di].y + A.at<double>(0,2);
        double py = A.at<double>(1,0)*detUndist[di].x + A.at<double>(1,1)*detUndist[di].y + A.at<double>(1,2);
        double dx = px - map[mi].x, dy = py - map[mi].y;
        errSum += std::sqrt(dx*dx + dy*dy);
    }
    p.reproj_err_m = p.n_inliers > 0
                   ? static_cast<float>(errSum / p.n_inliers) : 999.f;

    float reproj_px = p.reproj_err_m * intr.fx();  // meter → pixels

    // Verdict (dezelfde drempels als localise.py)
    if      (p.match_pct >= 70.f && reproj_px < 3.f) p.verdict = PoseResult::Verdict::Correct;
    else if (p.match_pct >= 50.f && reproj_px < 5.f) p.verdict = PoseResult::Verdict::Probable;
    else if (p.match_pct >= 30.f && reproj_px < 8.f) p.verdict = PoseResult::Verdict::Partial;
    else                                              p.verdict = PoseResult::Verdict::Doubtful;

    p.valid = (p.verdict != PoseResult::Verdict::Doubtful);
    return p;
}

// ── Inverteer 2×3 similarity matrix ──────────────────────────────────────────
// Zet om naar 3×3, inverteer, geef 2×3 terug.
inline cv::Mat invertSimilarity(const cv::Mat& A) {
    cv::Mat A3 = cv::Mat::eye(3, 3, CV_64F);
    A.copyTo(A3.rowRange(0, 2));
    cv::Mat inv = A3.inv();
    return inv.rowRange(0, 2).clone();
}

// Vereenvoudigde extractPose: alleen positie en rotatie, geen reprojection error.
// Gebruikt door Tracker voor delta-extractie.
inline PoseResult extractPose(const cv::Mat& A, const Intrinsics& intr) {
    PoseResult p;
    double cx = intr.cx(), cy = intr.cy();
    p.x_m  = static_cast<float>(A.at<double>(0,0)*cx + A.at<double>(0,1)*cy + A.at<double>(0,2));
    p.y_m  = static_cast<float>(A.at<double>(1,0)*cx + A.at<double>(1,1)*cy + A.at<double>(1,2));
    p.theta_deg = static_cast<float>(
        std::atan2(A.at<double>(1,0), A.at<double>(0,0)) * 180.0 / CV_PI);
    p.scale_est = static_cast<float>(
        std::sqrt(A.at<double>(0,0)*A.at<double>(0,0) +
                  A.at<double>(1,0)*A.at<double>(1,0)));
    return p;
}

// ── Homography decomposition → PoseResult ────────────────────────────────────
// Rs, ts, normals: output of cv::decomposeHomographyMat (up to 4 solutions).
// z_m: camera height above the plane IN METRES, derived from H's singular values
//      by the caller (Localiser) BEFORE this call. This is NOT a fixed calibration
//      constant — it is computed fresh from the data every frame, enabling free-camera
//      z tracking without a hardcoded height prior.
// inlierPairs, det, map: for reprojection error + match stats (may be empty).
//
// Solution selection: choose solution where normals[i].z > 0 (ceiling faces camera)
// AND ts[i].z > 0 (camera is on positive side of plane).
// If multiple valid solutions exist, pick the one with largest ts[i].z.
//
// Scale recovery: decomposeHomographyMat normalises t so that ||t|| ~ 1.
// The physical translation is: t_phys = ts[i] * (z_m / ts[i].at<double>(2)).
//
// Euler angles from R (ZYX convention: yaw about Z, pitch about Y, roll about X):
//   yaw   = atan2(R[1,0], R[0,0])
//   pitch = atan2(-R[2,0], sqrt(R[2,1]^2 + R[2,2]^2))
//   roll  = atan2(R[2,1], R[2,2])
inline PoseResult poseFromHomography(
        const std::vector<cv::Mat>& Rs,
        const std::vector<cv::Mat>& ts,
        const std::vector<cv::Mat>& normals,
        const Intrinsics& intr,
        float z_m,           // derived from H's SVD scale — NOT a fixed prior
        const std::vector<std::pair<int,int>>& inlierPairs,
        const std::vector<cv::Point2f>& det,
        const StarMap& map) {

    PoseResult p;

    // Select best valid solution
    int bestIdx = -1;
    double bestTz = -1e9;
    int n = static_cast<int>(Rs.size());
    for (int i = 0; i < n; ++i) {
        double nz = normals[i].at<double>(2);
        double tz = ts[i].at<double>(2);
        if (nz > 0.0 && tz > 0.0 && tz > bestTz) {
            bestTz = tz;
            bestIdx = i;
        }
    }

    if (bestIdx < 0) {
        p.valid = false;
        return p;  // no valid solution
    }

    const cv::Mat& R = Rs[bestIdx];
    const cv::Mat& t = ts[bestIdx];

    // Recover physical translation using z_m computed from H's singular values
    double tz_norm = t.at<double>(2);
    double scale   = static_cast<double>(z_m) / tz_norm;
    p.x_m = static_cast<float>(t.at<double>(0) * scale);
    p.y_m = static_cast<float>(t.at<double>(1) * scale);
    p.z_m = z_m;   // dynamic — changes with camera height

    // Euler angles (ZYX, degrees)
    double r00 = R.at<double>(0,0), r10 = R.at<double>(1,0), r20 = R.at<double>(2,0);
    double r21 = R.at<double>(2,1), r22 = R.at<double>(2,2);
    p.yaw_deg   = static_cast<float>(std::atan2(r10, r00) * 180.0 / CV_PI);
    p.pitch_deg = static_cast<float>(
        std::atan2(-r20, std::sqrt(r21*r21 + r22*r22)) * 180.0 / CV_PI);
    p.roll_deg  = static_cast<float>(std::atan2(r21, r22) * 180.0 / CV_PI);
    p.theta_deg = p.yaw_deg;  // keep legacy field in sync

    // Scale estimate (fx / z for current height)
    p.scale_est = static_cast<float>(intr.fx() / z_m);

    // Match statistics
    p.n_inliers    = static_cast<int>(inlierPairs.size());
    p.n_detections = static_cast<int>(det.size());
    p.match_pct    = p.n_detections > 0
                   ? 100.f * p.n_inliers / p.n_detections : 0.f;

    // Reprojection error: not computed here (done in Localiser after this call)
    p.reproj_err_m = 0.f;

    // Verdict (same thresholds as extractPose)
    float reproj_px = p.reproj_err_m * intr.fx();
    if      (p.match_pct >= 70.f && reproj_px < 3.f) p.verdict = PoseResult::Verdict::Correct;
    else if (p.match_pct >= 50.f && reproj_px < 5.f) p.verdict = PoseResult::Verdict::Probable;
    else if (p.match_pct >= 30.f && reproj_px < 8.f) p.verdict = PoseResult::Verdict::Partial;
    else if (p.n_inliers  >= 3)                       p.verdict = PoseResult::Verdict::Probable;
    else                                              p.verdict = PoseResult::Verdict::Doubtful;

    p.valid = (p.verdict != PoseResult::Verdict::Doubtful);
    return p;
}

} // namespace linalg
