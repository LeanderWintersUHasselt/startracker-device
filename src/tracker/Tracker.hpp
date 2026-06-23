#pragma once
#include "common/Types.hpp"
#include "common/Config.hpp"
#include "localiser/StarIndex.hpp"
#include <optional>
#include <vector>

// Lichtgewicht tracker: windowed NN-matching op een gekende pose.
// Geen RANSAC — de begrensdheid van de beweging garandeert correcte matches.
//
// Algoritme per frame:
//   1. Projecteer alle kaartsterren naar pixelruimte via A_prev⁻¹
//   2. Per kaartster: zoek dichtstbijzijnde detectie binnen maxPx
//   3. Fit similarity via gesloten-vorm least-squares (fitSimilarityLS)
//   4. Rotatiedelta-check: verwerp als |Δθ| > maxRotDeg
//   5. Inlier-filter op reprojectiefout
//   6. Herfit op inliers; extraheer delta (dx, dy, dθ) t.o.v. A_prev
//   7. Pas delta toe op lastAbsPose (x, y, yaw)
//
// z_m, roll_deg, pitch_deg worden ongewijzigd overgenomen van lastAbsPose.

class Tracker {
public:
    Tracker(Intrinsics intr, const StarIndex& index,
            float scaleHint, TrackerConfig cfg = {});

    struct Result {
        cv::Mat     A;     // bijgewerkte 2×3 similarity matrix (voor volgende frame)
        PoseResult  pose;  // incrementele absolute pose; pose.pose_confidence in [0,1]
    };

    // A_prev: 2×3 similarity matrix van het vorige frame (pixel→meter).
    // lastAbsPose: meest recente absolute pose van de Localiser.
    //   – z_m, roll_deg, pitch_deg worden ongewijzigd overgenomen.
    //   – x_m, y_m, yaw_deg worden incrementeel bijgewerkt met de similarity-delta.
    std::optional<Result> track(const std::vector<cv::Point2f>& detUndist,
                                const cv::Mat& A_prev,
                                const PoseResult& lastAbsPose) const;

private:
    Intrinsics       intr_;
    const StarIndex& index_;
    float            scaleHint_;
    TrackerConfig    cfg_;
};
