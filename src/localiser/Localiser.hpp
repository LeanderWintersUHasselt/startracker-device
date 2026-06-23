#pragma once
#include "common/Types.hpp"
#include "common/Config.hpp"
#include "localiser/StarIndex.hpp"
#include <optional>
#include <vector>

// Localiseert de camera t.o.v. een gekende sterkaart via:
//   1. Hash-vote: triplet-descriptors → kandidaat (det,map) paren
//   2. cv::findHomography (RANSAC ingebouwd) → H
//   3. cv::decomposeHomographyMat → R, t → volledige 6DOF pose

class Localiser {
public:
    Localiser(Intrinsics intr, const StarIndex& index,
              float scaleHint, LocaliserConfig cfg = {});

    // Hoofd-methode: geeft nullopt terug als localisatie mislukt.
    // detUndist: undistorted centroids (output van StarDetectorLight::detect).
    // height_m:  gekalibreerde plafondhoogte als fallback voor z-recovery.
    std::optional<PoseResult> localise(
        const std::vector<cv::Point2f>& detUndist,
        float height_m) const;

private:
    Intrinsics       intr_;
    const StarIndex& index_;
    float            scaleHint_;
    LocaliserConfig  cfg_;
};
