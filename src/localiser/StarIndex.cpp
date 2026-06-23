#include "localiser/StarIndex.hpp"
#include <algorithm>
#include <cmath>
#include <unordered_map>

// ── Hulpfuncties ──────────────────────────────────────────────────────────────

std::vector<int> StarIndex::kNearest(const StarMap& map, int i, int k) {
    int n = static_cast<int>(map.size());
    std::vector<std::pair<float,int>> dists;
    dists.reserve(n - 1);
    for (int j = 0; j < n; ++j) {
        if (j == i) continue;
        float dx = map[j].x - map[i].x;
        float dy = map[j].y - map[i].y;
        dists.emplace_back(dx*dx + dy*dy, j);
    }
    int kk = std::min(k, static_cast<int>(dists.size()));
    std::partial_sort(dists.begin(), dists.begin() + kk, dists.end());
    std::vector<int> result(kk);
    for (int t = 0; t < kk; ++t) result[t] = dists[t].second;
    return result;
}

bool StarIndex::tripletGeom(cv::Point2f c, cv::Point2f a, cv::Point2f b,
                             float& r1, float& r2, float& cosAngle) {
    float dca = std::hypot(a.x - c.x, a.y - c.y);
    float dcb = std::hypot(b.x - c.x, b.y - c.y);
    float dab = std::hypot(b.x - a.x, b.y - a.y);
    if (dab < 1e-9f) return false;

    r1 = dca / dab;
    r2 = dcb / dab;

    float dot = (a.x-c.x)*(b.x-c.x) + (a.y-c.y)*(b.y-c.y);
    float denom = dca * dcb;
    cosAngle = (denom > 1e-9f) ? (dot / denom) : 0.f;
    return true;
}

// ── Build ──────────────────────────────────────────────────────────────────────

StarIndex StarIndex::build(const StarMap& map, int k) {
    StarIndex idx;
    idx.map_ = map;
    int n = static_cast<int>(map.size());

    for (int ci = 0; ci < n; ++ci) {
        auto nbrs = kNearest(map, ci, k);
        int m = static_cast<int>(nbrs.size());
        // Alle paren (a,b) uit de buurenset
        for (int ia = 0; ia < m; ++ia) {
            for (int ib = ia + 1; ib < m; ++ib) {
                TripletEntry e;
                if (!tripletGeom(map[ci], map[nbrs[ia]], map[nbrs[ib]],
                                 e.r1, e.r2, e.cosAngle)) continue;
                e.centerIdx = ci;
                e.aIdx      = nbrs[ia];
                e.bIdx      = nbrs[ib];
                idx.entries_.push_back(e);
            }
        }
    }
    return idx;
}

// ── Query ─────────────────────────────────────────────────────────────────────

std::vector<std::pair<int,int>> StarIndex::query(
        const std::vector<cv::Point2f>& det,
        float scaleHint,
        float tol) const {
    // Bouw triplets voor de detecties (in pixels, schaal via scaleHint naar meter-ratio).
    int nd = static_cast<int>(det.size());
    if (nd < 3 || entries_.empty()) return {};

    // Stemtabel: (det_i, map_j) → gewogen stemgewicht
    std::unordered_map<int, float> votes; // key = det_i * 10000 + map_j

    int k = std::min(6, nd - 1);

    for (int ci = 0; ci < nd; ++ci) {
        // k dichtstbijzijnde buren van ci in de detectieset (pixel-eenheden)
        std::vector<std::pair<float,int>> dists;
        for (int j = 0; j < nd; ++j) {
            if (j == ci) continue;
            float dx = det[j].x - det[ci].x;
            float dy = det[j].y - det[ci].y;
            dists.emplace_back(dx*dx + dy*dy, j);
        }
        int kk = std::min(k, static_cast<int>(dists.size()));
        std::partial_sort(dists.begin(), dists.begin() + kk, dists.end());

        for (int ia = 0; ia < kk; ++ia) {
            for (int ib = ia + 1; ib < kk; ++ib) {
                float r1d, r2d, cosd;
                if (!tripletGeom(det[ci], det[dists[ia].second], det[dists[ib].second],
                                 r1d, r2d, cosd)) continue;

                // Vergelijk met alle map-entries (brute-force; 66 sterren → ~1k entries)
                for (const auto& e : entries_) {
                    if (std::abs(r1d - e.r1)       > tol) continue;
                    if (std::abs(r2d - e.r2)       > tol) continue;
                    if (std::abs(cosd - e.cosAngle) > tol) continue;

                    // Match: stem voor (ci, e.centerIdx)
                    int key = ci * 10000 + e.centerIdx;
                    votes[key] += 1.0f;
                }
            }
        }
    }

    // Sorteer op stemgewicht (aflopend)
    std::vector<std::pair<float,int>> ranked;
    ranked.reserve(votes.size());
    for (auto& [key, weight] : votes)
        ranked.emplace_back(weight, key);
    std::sort(ranked.begin(), ranked.end(), std::greater<>());

    // Verwijder conflicten: elke det-index slechts eenmaal (greedy)
    std::vector<std::pair<int,int>> result;
    std::vector<bool> usedDet(nd, false), usedMap(map_.size(), false);

    for (auto& [w, key] : ranked) {
        int di = key / 10000;
        int mi = key % 10000;
        if (di >= nd || mi >= static_cast<int>(map_.size())) continue;
        if (usedDet[di] || usedMap[mi]) continue;
        result.emplace_back(di, mi);
        usedDet[di]  = true;
        usedMap[mi]  = true;
    }
    return result;
}
