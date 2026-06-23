#include "localiser/Localiser.hpp"
#include "util/LinAlg.hpp"
#include <cmath>

Localiser::Localiser(Intrinsics intr, const StarIndex& index,
                     float scaleHint, LocaliserConfig cfg)
    : intr_(std::move(intr)), index_(index),
      scaleHint_(scaleHint), cfg_(cfg) {}

std::optional<PoseResult> Localiser::localise(
        const std::vector<cv::Point2f>& detUndist,
        float height_m) const {
    if (detUndist.size() < 4) return std::nullopt;

    const StarMap& map = index_.map();

    // ── 1. Kandidaat-paren ophalen (hash of brute-force fallback) ─────────────
    std::vector<std::pair<int,int>> candidates;
    if (cfg_.useHash)
        candidates = index_.query(detUndist, scaleHint_);

    if (candidates.size() < 4) {
        candidates.clear();
        int nd = static_cast<int>(detUndist.size());
        int nm = static_cast<int>(map.size());
        // Nearest-neighbour brute-force: for each detection, find the closest
        // map star in projected pixel space and emit that single pair.
        // This is deterministic, avoids the symmetric-constellation ambiguity
        // that all-pairs RANSAC suffers from, and is correct when the camera is
        // roughly positioned over the mapped area.
        float fx_ = intr_.fx(), fy_ = intr_.fy(), cx_ = intr_.cx(), cy_ = intr_.cy();
        for (int di = 0; di < nd; ++di) {
            float dpx = detUndist[di].x, dpy = detUndist[di].y;
            float bestD2 = std::numeric_limits<float>::max();
            int   bestMi = -1;
            for (int mi = 0; mi < nm; ++mi) {
                float mpx = map[mi].x * fx_ + cx_;
                float mpy = map[mi].y * fy_ + cy_;
                float dx = dpx - mpx, dy = dpy - mpy;
                float d2 = dx*dx + dy*dy;
                if (d2 < bestD2) { bestD2 = d2; bestMi = mi; }
            }
            if (bestMi >= 0)
                candidates.emplace_back(di, bestMi);
        }
        // Last resort: if NN gives too few, use all-pairs.
        if (candidates.size() < 4) {
            candidates.clear();
            candidates.reserve(static_cast<size_t>(nd * nm));
            for (int di = 0; di < nd; ++di)
                for (int mi = 0; mi < nm; ++mi)
                    candidates.emplace_back(di, mi);
        }
    }

    if (cfg_.verbose)
        std::printf("[LOC] det=%d  candidates=%d\n",
                    (int)detUndist.size(), (int)candidates.size());

    if (candidates.size() < 4) return std::nullopt;

    // ── 2. Bouw punt-arrays voor findHomography ───────────────────────────────
    // findHomography: src = detectie-pixels, dst = kaartpunten als "virtuele pixels"
    // via K (map_m * fx + cx). Dit behandelt de kaart als geprojecteerd op z_ref=1m.
    std::vector<cv::Point2f> srcPts, dstPts;
    srcPts.reserve(candidates.size());
    dstPts.reserve(candidates.size());

    float fx = intr_.fx(), fy = intr_.fy(), cx = intr_.cx(), cy = intr_.cy();
    for (auto& [di, mi] : candidates) {
        srcPts.push_back(detUndist[di]);
        dstPts.push_back({map[mi].x * fx + cx, map[mi].y * fy + cy});
    }

    // ── 3. findHomography (RANSAC ingebouwd) ──────────────────────────────────
    // A valid camera-above-plane homography must have det(H[0:2,0:2]) > 0
    // (no reflections). Run up to cfg_.ransacIter/1000 + 3 trials and keep
    // the first non-reflection result. This makes the search robust to the
    // symmetric-constellation ambiguity that can fool a single RANSAC run.
    cv::Mat H;
    cv::Mat inlierMask;
    {
        int maxTrials = std::max(3, cfg_.ransacIter / 1000);
        for (int trial = 0; trial < maxTrials; ++trial) {
            cv::Mat maskTrial;
            cv::Mat Htrial = cv::findHomography(srcPts, dstPts, cv::RANSAC,
                                                cfg_.threshPx, maskTrial);
            if (Htrial.empty()) continue;
            double det2x2 = cv::determinant(Htrial.rowRange(0,2).colRange(0,2));
            if (det2x2 > 0.0) {
                H           = Htrial;
                inlierMask  = maskTrial;
                break;
            }
        }
    }
    if (H.empty()) {
        if (cfg_.verbose)
            std::printf("[LOC] findHomography: geen geldige (niet-reflectie) oplossing\n");
        return std::nullopt;
    }

    // Inlier-paren verzamelen
    std::vector<std::pair<int,int>> inlierPairs;
    for (int i = 0; i < (int)candidates.size(); ++i) {
        if (inlierMask.at<uchar>(i))
            inlierPairs.emplace_back(candidates[i]);
    }

    if (cfg_.verbose)
        std::printf("[LOC] H gevonden: %d inliers van %d candidates\n",
                    (int)inlierPairs.size(), (int)candidates.size());

    if (inlierPairs.size() < 3) return std::nullopt;

    // ── 4. z herleiden uit de schaal van H ────────────────────────────────────
    // dst = map_m * fx + cx  →  behandelt map als geprojecteerd op z_ref = 1m
    // Voor een niveau camera op hoogte z: H ≈ [[z,0,cx(1-z)],[0,z,cy(1-z)],[0,0,1]]
    // De singuliere waarden van de bovenlinker 2×2 van H_normalized zijn beide ≈ z.
    float z_m = height_m;  // fallback als SVD faalt of tilt te groot
    {
        cv::Mat H_norm = H / H.at<double>(2, 2);
        cv::Mat H22    = H_norm.rowRange(0, 2).colRange(0, 2).clone();
        cv::Mat W, U, Vt;
        cv::SVDecomp(H22, W, U, Vt);
        double sv1 = W.at<double>(0), sv2 = W.at<double>(1);
        if (sv1 > 0.01 && sv2 > 0.01) {
            double z_svd    = std::sqrt(sv1 * sv2);
            double sv_ratio = sv1 / sv2;  // near 1.0 = level, large = tilted
            // Tilt guard: sv_ratio >= 2.0 → significant tilt → keep fallback
            if (z_svd > 0.3 && z_svd < 10.0 && sv_ratio < 2.0)
                z_m = static_cast<float>(z_svd);
        }
        if (cfg_.verbose)
            std::printf("[LOC] z_m=%.3fm (SVD: sv1=%.4f sv2=%.4f fallback=%.3f)\n",
                        z_m, sv1, sv2, (float)height_m);
    }

    // ── 5. Pose extraheren direct uit H (x, y, z, yaw) ──────────────────────────
    // K^{-1} * H_norm * K geeft de affiene kaartprojectie:
    //   [[z·cos θ, -z·sin θ, x_m],
    //    [z·sin θ,  z·cos θ, y_m],
    //    [~0,       ~0,      1  ]]
    // x_m en y_m zijn de cameracoördinaten in de sterkaart (meter).
    // Dit is exact voor een niveau camera en een goede benadering voor lichte kanteling.
    cv::Mat H_norm  = H / H.at<double>(2, 2);
    cv::Mat Hp      = intr_.K.inv() * H_norm * intr_.K;  // affiene kaartprojectie

    PoseResult pose;
    pose.x_m       = static_cast<float>(Hp.at<double>(0, 2));
    pose.y_m       = static_cast<float>(Hp.at<double>(1, 2));
    pose.z_m       = -z_m;  // tracker world z (negative below ceiling)
    pose.scale_est = static_cast<float>(intr_.fx() / z_m);

    // Yaw: uit de rotatie-component van Hp (bovenlinks 2×2 ≈ z · R_yaw)
    {
        double s00 = Hp.at<double>(0, 0), s10 = Hp.at<double>(1, 0);
        double sc  = std::sqrt(s00*s00 + s10*s10);
        pose.yaw_deg   = (sc > 1e-6) ? static_cast<float>(
            std::atan2(s10/sc, s00/sc) * 180.0 / CV_PI) : 0.f;
        pose.theta_deg = pose.yaw_deg;  // legacy alias
    }

    // Statistieken
    pose.n_inliers    = static_cast<int>(inlierPairs.size());
    pose.n_detections = static_cast<int>(detUndist.size());
    pose.match_pct    = pose.n_detections > 0
                      ? 100.f * pose.n_inliers / (float)pose.n_detections : 0.f;

    // ── 6. Roll en pitch uit decompositie ─────────────────────────────────────
    // decomposeHomographyMat levert de volledige R voor kleine kanteling.
    // Selecteer de oplossing met tz > 0 (vlak voor de camera).
    // Als decompositie mislukt (sigma1≈sigma2 voor niveau camera), blijven roll/pitch 0.
    std::vector<cv::Mat> Rs, ts, normals;
    int numSol = cv::decomposeHomographyMat(H, intr_.K, Rs, ts, normals);
    for (int i = 0; i < numSol; ++i) {
        if (ts[i].at<double>(2) > 0.0) {
            const cv::Mat& R = Rs[i];
            double r20 = R.at<double>(2, 0);
            double r21 = R.at<double>(2, 1), r22 = R.at<double>(2, 2);
            pose.pitch_deg = static_cast<float>(
                std::atan2(-r20, std::sqrt(r21*r21 + r22*r22)) * 180.0 / CV_PI);
            pose.roll_deg  = static_cast<float>(std::atan2(r21, r22) * 180.0 / CV_PI);
            break;
        }
    }

    // ── 7. Reprojectiefout berekenen ──────────────────────────────────────────
    // H mapt det-pixels → kaart-pixels. Project via H, converteer naar meter.
    double errSum = 0.0;
    for (auto& [di, mi] : inlierPairs) {
        std::vector<cv::Point2f> src_pt = {detUndist[di]};
        std::vector<cv::Point2f> proj_pt;
        cv::perspectiveTransform(src_pt, proj_pt, H);  // det-px → kaart-px
        float px_m = (proj_pt[0].x - cx) / fx;
        float py_m = (proj_pt[0].y - cy) / fy;
        float dx = px_m - map[mi].x;
        float dy = py_m - map[mi].y;
        errSum += std::sqrt(dx*dx + dy*dy);
    }
    pose.reproj_err_m = inlierPairs.empty() ? 999.f
                      : static_cast<float>(errSum / inlierPairs.size());

    // Herbereken verdict met echte reproj fout
    float reproj_px = pose.reproj_err_m * intr_.fx();
    if      (pose.match_pct >= 70.f && reproj_px < 3.f) pose.verdict = PoseResult::Verdict::Correct;
    else if (pose.match_pct >= 50.f && reproj_px < 5.f) pose.verdict = PoseResult::Verdict::Probable;
    else if (pose.match_pct >= 30.f && reproj_px < 8.f) pose.verdict = PoseResult::Verdict::Partial;
    else                                                 pose.verdict = PoseResult::Verdict::Doubtful;
    pose.valid = (pose.verdict != PoseResult::Verdict::Doubtful);

    // ── 8. Confidence en relocalisation flag ──────────────────────────────────
    float inlier_ratio = std::min(pose.match_pct / 100.f, 1.f);
    float err_factor   = std::max(0.f, 1.f - pose.reproj_err_m / 0.05f);
    pose.pose_confidence = 0.6f * inlier_ratio + 0.4f * err_factor;
    pose.relocalisation_requested = false;  // Localiser itself doesn't set this

    return pose;
}
