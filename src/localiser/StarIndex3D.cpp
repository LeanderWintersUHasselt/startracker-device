#include "localiser/StarIndex3D.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

std::vector<int> StarIndex3D::kNearest(const StarMap3D& map, int i, int k) {
    int n = static_cast<int>(map.size());
    std::vector<std::pair<float,int>> dists;
    dists.reserve(n - 1);
    for (int j = 0; j < n; ++j) {
        if (j == i) continue;
        float dx = map[j].p_world_m.x - map[i].p_world_m.x;
        float dy = map[j].p_world_m.y - map[i].p_world_m.y;
        dists.emplace_back(dx*dx + dy*dy, j);
    }
    int kk = std::min(k, static_cast<int>(dists.size()));
    std::partial_sort(dists.begin(), dists.begin() + kk, dists.end());
    std::vector<int> result(kk);
    for (int t = 0; t < kk; ++t) result[t] = dists[t].second;
    return result;
}

bool StarIndex3D::tripletGeom(cv::Point2f c, cv::Point2f a, cv::Point2f b,
                               float& r1, float& r2, float& cosAngle) {
    float dca = std::hypot(a.x - c.x, a.y - c.y);
    float dcb = std::hypot(b.x - c.x, b.y - c.y);
    float dab = std::hypot(b.x - a.x, b.y - a.y);
    if (dab < 1e-9f) return false;

    r1 = dca / dab;
    r2 = dcb / dab;

    float dot   = (a.x-c.x)*(b.x-c.x) + (a.y-c.y)*(b.y-c.y);
    float denom = dca * dcb;
    cosAngle = (denom > 1e-9f) ? (dot / denom) : 0.f;
    return true;
}

StarIndex3D StarIndex3D::build(const StarMap3D& map3d, int k) {
    StarIndex3D idx;
    idx.map3d_ = map3d;
    int n = static_cast<int>(map3d.size());

    for (int ci = 0; ci < n; ++ci) {
        auto nbrs = kNearest(map3d, ci, k);
        int m = static_cast<int>(nbrs.size());
        for (int ia = 0; ia < m; ++ia) {
            for (int ib = ia + 1; ib < m; ++ib) {
                TripletEntry3D e;
                cv::Point2f pc{map3d[ci].p_world_m.x, map3d[ci].p_world_m.y};
                cv::Point2f pa{map3d[nbrs[ia]].p_world_m.x, map3d[nbrs[ia]].p_world_m.y};
                cv::Point2f pb{map3d[nbrs[ib]].p_world_m.x, map3d[nbrs[ib]].p_world_m.y};
                if (!tripletGeom(pc, pa, pb, e.r1, e.r2, e.cosAngle)) continue;
                e.centerIdx = ci;
                e.aIdx      = nbrs[ia];
                e.bIdx      = nbrs[ib];
                idx.entries_.push_back(e);
            }
        }
    }
    return idx;
}

std::vector<std::pair<int,int>> StarIndex3D::query(
        const std::vector<cv::Point2f>& det,
        float tol) const {
    int nd = static_cast<int>(det.size());
    if (nd < 3 || entries_.empty()) return {};

    std::unordered_map<uint64_t, float> votes;
    int k = std::min(6, nd - 1);

    for (int ci = 0; ci < nd; ++ci) {
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

                for (const auto& e : entries_) {
                    if (std::abs(r1d - e.r1)        > tol) continue;
                    if (std::abs(r2d - e.r2)        > tol) continue;
                    if (std::abs(cosd - e.cosAngle) > tol) continue;
                    uint64_t key = (static_cast<uint64_t>(ci) << 32) |
                                    static_cast<uint32_t>(e.centerIdx);
                    votes[key] += 1.0f;
                }
            }
        }
    }

    std::vector<std::pair<float,uint64_t>> ranked;
    ranked.reserve(votes.size());
    for (auto& [key, weight] : votes)
        ranked.emplace_back(weight, key);
    std::sort(ranked.begin(), ranked.end(), std::greater<>());

    std::vector<std::pair<int,int>> result;
    std::vector<bool> usedDet(nd, false), usedMap(map3d_.size(), false);

    for (auto& [w, key] : ranked) {
        int di = static_cast<int>(key >> 32);
        int mi = static_cast<int>(key & 0xFFFFFFFF);
        if (di >= nd || mi >= static_cast<int>(map3d_.size())) continue;
        if (usedDet[di] || usedMap[mi]) continue;
        result.emplace_back(di, mi);
        usedDet[di]  = true;
        usedMap[mi]  = true;
    }
    return result;
}
