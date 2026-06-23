#pragma once
#include "common/Types.hpp"
#include <vector>

// Scale- and rotation-invariant triplet descriptor database for StarMap3D.
//
// Triplet descriptor for markers c, a, b from the metric map:
//   (d_ca/d_ab, d_cb/d_ab, cos_angle_at_c)
// Using metric distances makes descriptors independent of image scale.
//
// Query: given detected pixel positions (undistorted or raw), return the best
// (detection_idx, map_marker_idx) candidate pairs ranked by vote weight.
// Because ratios are scale-invariant the same descriptor matches at any
// camera height; no scaleHint parameter needed.

struct TripletEntry3D {
    float r1, r2, cosAngle;  // descriptor
    int   centerIdx;         // marker index in StarMap3D
    int   aIdx, bIdx;
};

class StarIndex3D {
public:
    // Build the index from a StarMap3D.  Only X,Y of p_world_m are used (Z=0).
    static StarIndex3D build(const StarMap3D& map3d, int k = 6);

    // Query: returns (detection_idx, map_marker_idx) pairs sorted by vote weight.
    // detPoints: 2D pixel positions of detected stars (undistorted or raw — ratios
    //   are invariant to the distortion-space as long as points are consistent).
    std::vector<std::pair<int,int>> query(
        const std::vector<cv::Point2f>& detPoints,
        float tolerance = 0.12f) const;

    const StarMap3D& map3d() const { return map3d_; }
    size_t           size()  const { return map3d_.size(); }

private:
    StarMap3D                   map3d_;
    std::vector<TripletEntry3D> entries_;

    static std::vector<int> kNearest(const StarMap3D& map, int i, int k);
    static bool tripletGeom(cv::Point2f c, cv::Point2f a, cv::Point2f b,
                            float& r1, float& r2, float& cosAngle);
};
