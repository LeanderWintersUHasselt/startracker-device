#include "tracker/Tracker.hpp"
#include "util/LinAlg.hpp"
#include <cmath>

Tracker::Tracker(Intrinsics intr, const StarIndex& index,
                 float scaleHint, TrackerConfig cfg)
    : intr_(std::move(intr)), index_(index),
      scaleHint_(scaleHint), cfg_(cfg) {}

std::optional<Tracker::Result> Tracker::track(
        const std::vector<cv::Point2f>& det,
        const cv::Mat& A_prev,
        const PoseResult& lastAbsPose) const {
    const StarMap& map = index_.map();
    int nm = static_cast<int>(map.size());
    int nd = static_cast<int>(det.size());
    if (nd < 3) return std::nullopt;

    // ── 1. Projecteer kaartsterren naar pixelruimte via A_prev⁻¹ (meter→pixel) ─
    cv::Mat A_inv = linalg::invertSimilarity(A_prev);  // 2×3 meter→pixel

    std::vector<cv::Point2f> mapPx(nm);
    for (int mi = 0; mi < nm; ++mi) {
        double x = map[mi].x, y = map[mi].y;
        mapPx[mi].x = static_cast<float>(
            A_inv.at<double>(0,0)*x + A_inv.at<double>(0,1)*y + A_inv.at<double>(0,2));
        mapPx[mi].y = static_cast<float>(
            A_inv.at<double>(1,0)*x + A_inv.at<double>(1,1)*y + A_inv.at<double>(1,2));
    }

    // ── 2. Windowed NN: per kaartster dichtstbijzijnde detectie binnen maxPx ──
    float maxPx2 = cfg_.maxPx * cfg_.maxPx;
    std::vector<std::pair<int,int>> pairs; // (det_i, map_j)
    std::vector<bool> usedDet(nd, false);

    for (int mi = 0; mi < nm; ++mi) {
        float bestD2 = maxPx2;
        int   bestDi = -1;
        for (int di = 0; di < nd; ++di) {
            if (usedDet[di]) continue;
            float dx = det[di].x - mapPx[mi].x;
            float dy = det[di].y - mapPx[mi].y;
            float d2 = dx*dx + dy*dy;
            if (d2 < bestD2) { bestD2 = d2; bestDi = di; }
        }
        if (bestDi >= 0) {
            pairs.emplace_back(bestDi, mi);
            usedDet[bestDi] = true;
        }
    }

    if (pairs.size() < 3) return std::nullopt;

    // ── 3. Similarity fit op alle gevonden paren ──────────────────────────────
    std::vector<cv::Point2f> src, dst;
    src.reserve(pairs.size());
    dst.reserve(pairs.size());
    for (auto& [di, mi] : pairs) {
        src.push_back(det[di]);
        dst.push_back(map[mi]);
    }

    auto A_new = linalg::fitSimilarityLS(src, dst);
    if (!A_new) return std::nullopt;

    // ── 4. Rotatiedelta-check ─────────────────────────────────────────────────
    double theta_prev = std::atan2(A_prev.at<double>(1,0), A_prev.at<double>(0,0));
    double theta_new  = std::atan2(A_new->at<double>(1,0), A_new->at<double>(0,0));
    double delta      = std::abs(theta_new - theta_prev) * 180.0 / CV_PI;
    if (delta > 180.0) delta = 360.0 - delta;
    if (delta > cfg_.maxRotDeg) return std::nullopt;

    // ── 5. Inlier-filter op reprojectiefout ───────────────────────────────────
    float thresh_m  = cfg_.maxPx / scaleHint_ * 1.5f;
    float thresh_m2 = thresh_m * thresh_m;

    std::vector<std::pair<int,int>> inlierPairs;
    std::vector<cv::Point2f> src2, dst2;
    for (auto& [di, mi] : pairs) {
        double px = A_new->at<double>(0,0)*det[di].x + A_new->at<double>(0,1)*det[di].y + A_new->at<double>(0,2);
        double py = A_new->at<double>(1,0)*det[di].x + A_new->at<double>(1,1)*det[di].y + A_new->at<double>(1,2);
        float dx = static_cast<float>(px) - map[mi].x;
        float dy = static_cast<float>(py) - map[mi].y;
        if (dx*dx + dy*dy < thresh_m2) {
            inlierPairs.emplace_back(di, mi);
            src2.push_back(det[di]);
            dst2.push_back(map[mi]);
        }
    }

    if (inlierPairs.size() < 3) return std::nullopt;

    // ── 6. Herfit op inliers ───────────────────────────────────────────────────
    if (src2.size() < pairs.size()) {
        auto refined = linalg::fitSimilarityLS(src2, dst2);
        if (refined) A_new = refined;
    }

    // ── 7. Delta extracteren t.o.v. A_prev ───────────────────────────────────
    // Gebruik de vereenvoudigde 2-parameter extractPose (geen reprojection error).
    PoseResult prevP = linalg::extractPose(A_prev,  intr_);
    PoseResult newP  = linalg::extractPose(*A_new, intr_);

    float dx_m     = newP.x_m       - prevP.x_m;
    float dy_m     = newP.y_m       - prevP.y_m;
    float dtheta   = newP.theta_deg - prevP.theta_deg;

    // ── 8. Delta toepassen op lastAbsPose ─────────────────────────────────────
    // x, y, yaw worden incrementeel bijgewerkt.
    // z_m, roll_deg, pitch_deg worden ongewijzigd overgenomen.
    PoseResult pose;
    pose.x_m       = lastAbsPose.x_m       + dx_m;
    pose.y_m       = lastAbsPose.y_m       + dy_m;
    pose.yaw_deg   = lastAbsPose.yaw_deg   + dtheta;
    pose.theta_deg = pose.yaw_deg;  // legacy alias in sync
    pose.z_m       = lastAbsPose.z_m;
    pose.roll_deg  = lastAbsPose.roll_deg;
    pose.pitch_deg = lastAbsPose.pitch_deg;
    pose.scale_est = newP.scale_est;

    // ── 9. Statistieken en kwaliteitscheck ───────────────────────────────────
    pose.n_inliers    = static_cast<int>(inlierPairs.size());
    pose.n_detections = nd;
    pose.match_pct    = 100.f * pose.n_inliers / static_cast<float>(nd);

    // Reprojectiefout via A_new (inlier herfit)
    double errSum = 0.0;
    for (auto& [di, mi] : inlierPairs) {
        double px = A_new->at<double>(0,0)*det[di].x + A_new->at<double>(0,1)*det[di].y + A_new->at<double>(0,2);
        double py = A_new->at<double>(1,0)*det[di].x + A_new->at<double>(1,1)*det[di].y + A_new->at<double>(1,2);
        double ex = px - map[mi].x, ey = py - map[mi].y;
        errSum += std::sqrt(ex*ex + ey*ey);
    }
    pose.reproj_err_m = static_cast<float>(errSum / inlierPairs.size());

    if (pose.match_pct < cfg_.minMatchPct) return std::nullopt;

    // Confidence: gewogen combinatie van inlier-ratio en reprojectiefout
    float inlier_ratio = std::min(pose.match_pct / 100.f, 1.f);
    float err_factor   = std::max(0.f, 1.f - pose.reproj_err_m / cfg_.maxReprojM);
    pose.pose_confidence = 0.6f * inlier_ratio + 0.4f * err_factor;

    // Relocalisation trigger: reprojectiefout boven drempel → vraag herlocalisatie
    pose.relocalisation_requested = (pose.reproj_err_m > cfg_.maxReprojM);

    // Verdict
    float reproj_px = pose.reproj_err_m * intr_.fx();
    if      (pose.match_pct >= 70.f && reproj_px < 3.f) pose.verdict = PoseResult::Verdict::Correct;
    else if (pose.match_pct >= 50.f && reproj_px < 5.f) pose.verdict = PoseResult::Verdict::Probable;
    else if (pose.match_pct >= 30.f && reproj_px < 8.f) pose.verdict = PoseResult::Verdict::Partial;
    else                                                 pose.verdict = PoseResult::Verdict::Doubtful;
    pose.valid = (pose.verdict != PoseResult::Verdict::Doubtful);

    return Result{ *A_new, pose };
}
