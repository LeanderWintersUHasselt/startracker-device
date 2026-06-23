#pragma once
#include "common/Types.hpp"
#include <vector>
#include <string>

// Schaal- en rotatie-invariante triplet-descriptor database.
// Directe port van star_index.py.
//
// Per ster c met k buren worden alle paren (a,b) uit die buren beschouwd.
// Descriptor: (d_ca/d_ab, d_cb/d_ab, cos_hoek_bij_c)
//   – schaalinvariant door ratio's van afstanden
//   – rotatie-invariant door cosinus
//
// Query: geef voor een set gedetecteerde undistorted punten de beste
//        (det_i, map_j) kandidaat-paren terug gesorteerd op aantal stemmen.

struct TripletEntry {
    float r1, r2, cosAngle;  // descriptor
    int   centerIdx;         // ster-index in de map
    int   aIdx, bIdx;        // burenindices
};

class StarIndex {
public:
    // Bouw index vanuit een sterkaart (alle triplets voor k buren per ster).
    static StarIndex build(const StarMap& map, int k = 6);

    // Geeft kandidaat-(det_i, map_j)-paren terug gesorteerd op stemgewicht.
    // detUndist: undistorted pixel-posities van gedetecteerde sterren.
    // scaleHint: verwachte schaal (px/m) voor descriptor-normalisatie.
    std::vector<std::pair<int,int>> query(
        const std::vector<cv::Point2f>& detUndist,
        float scaleHint,
        float tolerance = 0.10f) const;

    const StarMap& map()  const { return map_; }
    size_t         size() const { return map_.size(); }

private:
    StarMap                    map_;
    std::vector<TripletEntry>  entries_;

    // k dichtstbijzijnde buren van ster i (brute-force, snel voor ≤200 sterren).
    static std::vector<int> kNearest(const StarMap& map, int i, int k);

    // Triplet descriptor voor sterren c, a, b (returnwaarde is geldig als true).
    static bool tripletGeom(cv::Point2f c, cv::Point2f a, cv::Point2f b,
                             float& r1, float& r2, float& cosAngle);
};
