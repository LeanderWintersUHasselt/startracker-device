#include "detector/StarDetector.hpp"
#include <algorithm>
#include <cmath>

StarDetector::StarDetector(DetectorConfig cfg) : cfg_(cfg) {
    // Bouw de morfologische kernel eenmalig bij constructie.
    int k = (cfg_.morphKernel % 2 == 0) ? cfg_.morphKernel + 1 : cfg_.morphKernel;
    kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, {k, k});
}

// ── detectRawCentroids ────────────────────────────────────────────────────────
// Returns centroids in the input image's pixel space — no distortion correction.

std::vector<cv::Point2f> StarDetector::detectRawCentroids(const cv::Mat& bgr) const {
    // ── 1. BGR → HSV ──────────────────────────────────────────────────────────
    cv::cvtColor(bgr, hsv_, cv::COLOR_BGR2HSV);

    // ── 2. White mask: laag S én hoog V (identiek aan Python inRange) ─────────
    std::vector<cv::Mat> channels(3);
    cv::split(hsv_, channels);
    const cv::Mat& S = channels[1];
    const cv::Mat& V = channels[2];     // bewaar referentie voor achtergrondaftrek

    cv::threshold(S, chroma_, cfg_.satMax, 255, cv::THRESH_BINARY_INV);  // S ≤ satMax
    cv::threshold(V, val_,    cfg_.valMin, 255, cv::THRESH_BINARY);      // V ≥ valMin
    cv::bitwise_and(chroma_, val_, white_mask_);                          // combinatie

    // ── 3. Achtergrondaftrek: morphologische OPEN op 2× downsampled V ─────────
    // bgKernel <= 1: geen achtergrondaftrek.
    // bgKernel > 1: downsample V 2× voor de OPEN → 4× minder pixels, zelfde
    //   ruimtelijke schaal. bgKernel wordt geïnterpreteerd als full-res equivalent;
    //   half-res gebruikt bgKernel/2. Resultaat wordt teruggeschaald voor aftrek.
    if (cfg_.bgKernel > 1) {
        cv::Mat V_ds;
        cv::resize(V, V_ds, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        int bgK_half = std::max(3, cfg_.bgKernel / 2);
        if (bgK_half % 2 == 0) ++bgK_half;
        cv::Mat bgKernelMat = cv::getStructuringElement(cv::MORPH_ELLIPSE, {bgK_half, bgK_half});
        cv::morphologyEx(V_ds, bg_, cv::MORPH_OPEN, bgKernelMat);
        cv::Mat bg_full;
        cv::resize(bg_, bg_full, V.size(), 0, 0, cv::INTER_LINEAR);
        cv::subtract(V, bg_full, enhanced_);
    } else {
        enhanced_ = V;
    }

    // ── 4. Drempel op enhanced: Otsu-adaptief met peakFloor als minimum ───────
    // Python: peak_thr = max(otsu_val, self.peak_floor)
    cv::Mat otsuTmp;
    double otsuVal = cv::threshold(enhanced_, otsuTmp,
                                   0, 255,
                                   cv::THRESH_BINARY | cv::THRESH_OTSU);
    double peakThr = std::max(static_cast<double>(cfg_.peakFloor), otsuVal);
    if (cfg_.peakCeil < 255) peakThr = std::min(peakThr, static_cast<double>(cfg_.peakCeil));
    cv::threshold(enhanced_, thresh_, peakThr, 255, cv::THRESH_BINARY);
    last_otsu_val_    = otsuVal;
    n_total_contours_ = 0;
    n_rejected_area_  = 0;
    n_rejected_circ_  = 0;

    // Combineer met white mask
    cv::bitwise_and(thresh_, white_mask_, mask_final_);

    // ── 5. Morph overgeslagen: area_min + circ_min filteren ruis al op contourniveau ─
    mask_clean_ = mask_final_;   // alias zodat debug-grid klopt

    // ── 6. Contouren + filter ─────────────────────────────────────────────────
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_final_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    auto passes = [&](const std::vector<cv::Point>& cnt, cv::Point2f& out) -> bool {
        ++n_total_contours_;
        float area = static_cast<float>(cv::contourArea(cnt));
        if (area < cfg_.areaMin || area > cfg_.areaMax) { ++n_rejected_area_; return false; }
        float perim = static_cast<float>(cv::arcLength(cnt, true));
        if (perim < 1e-3f) return false;
        float circ = 4.f * static_cast<float>(CV_PI) * area / (perim * perim);
        if (circ < cfg_.circMin) { ++n_rejected_circ_; return false; }

        cv::Rect bbox = cv::boundingRect(cnt)
                      & cv::Rect(0, 0, V.cols, V.rows);
        if (bbox.empty()) return false;
        cv::Mat blob_mask = cv::Mat::zeros(bbox.size(), CV_8U);
        std::vector<cv::Point> cnt_s;
        cnt_s.reserve(cnt.size());
        for (const auto& p : cnt)
            cnt_s.push_back({ p.x - bbox.x, p.y - bbox.y });
        cv::fillPoly(blob_mask, std::vector<std::vector<cv::Point>>{cnt_s}, 255);
        cv::Mat roi = cv::Mat::zeros(bbox.size(), CV_8U);
        V(bbox).copyTo(roi, blob_mask);
        cv::Moments m = cv::moments(roi, false);
        if (m.m00 < 1e-6) return false;
        out = {
            static_cast<float>(m.m10 / m.m00) + bbox.x,
            static_cast<float>(m.m01 / m.m00) + bbox.y
        };
        return true;
    };

    std::vector<cv::Point2f> centroids;
    accepted_ = cv::Mat::zeros(mask_clean_.size(), CV_8U);
    for (const auto& cnt : contours) {
        cv::Point2f centre;
        if (!passes(cnt, centre)) continue;
        centroids.push_back(centre);
        cv::drawContours(accepted_, std::vector<std::vector<cv::Point>>{cnt}, -1, 255, cv::FILLED);
    }

    return centroids;
}

// ── detectUndistortedCentroids ────────────────────────────────────────────────

std::vector<cv::Point2f> StarDetector::detectUndistortedCentroids(
        const cv::Mat& bgr, const cv::Mat& K, const cv::Mat& dist) const {
    auto centroids = detectRawCentroids(bgr);
    if (centroids.empty()) return {};

    cv::Mat pts(centroids, true);
    pts = pts.reshape(2, static_cast<int>(centroids.size()));
    cv::Mat undist;
    cv::undistortPoints(pts, undist, K, dist, cv::noArray(), K);
    undist = undist.reshape(2);

    std::vector<cv::Point2f> result;
    result.reserve(centroids.size());
    for (int i = 0; i < undist.rows; ++i)
        result.push_back(undist.at<cv::Point2f>(i));
    return result;
}

// ── detect (legacy alias) ─────────────────────────────────────────────────────

std::vector<cv::Point2f> StarDetector::detect(const cv::Mat& bgr,
                                               const Intrinsics& intr) const {
    return detectUndistortedCentroids(bgr, intr.K, intr.dist);
}

// ── makeDebugGrid ─────────────────────────────────────────────────────────────

cv::Mat StarDetector::makeDebugGrid(const cv::Mat& bgr,
                                     const std::vector<cv::Point2f>& dets,
                                     int tileW, int tileH) const {
    cv::Size tile(tileW, tileH);

    // Helper: schaal willekeurige Mat naar tile, zet om naar BGR
    auto toTile = [&](const cv::Mat& src, int code = -1) -> cv::Mat {
        if (src.empty()) return cv::Mat(tile, CV_8UC3, cv::Scalar(40,40,40));
        cv::Mat tmp, out;
        if (code >= 0) cv::cvtColor(src, tmp, code);
        else           tmp = src;
        // normaliseer 8U als het geen 8U is
        if (tmp.depth() != CV_8U) cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8U);
        // naar BGR
        if (tmp.channels() == 1) cv::cvtColor(tmp, out, cv::COLOR_GRAY2BGR);
        else out = tmp.clone();
        cv::resize(out, out, tile);
        return out;
    };

    // Tegel 1: Ruw BGR
    cv::Mat t1 = toTile(bgr);
    cv::putText(t1, "1 RAW", {8,28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,255}, 2);

    // Tegel 2: Value-kanaal (helderheid)
    cv::Mat valBgr = toTile(val_);
    cv::putText(valBgr, "2 VALUE (V)", {8,28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,255}, 2);

    // Tegel 3: Enhanced (V - blur(V))
    cv::Mat enhBgr = toTile(enhanced_);
    cv::putText(enhBgr, "3 ENHANCED", {8,28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,255}, 2);

    // Tegel 4: Masker na drempelstelling + chroma
    cv::Mat mfBgr = toTile(mask_final_);
    cv::putText(mfBgr, "4 MASK (thresh+chroma)", {8,28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,255}, 2);

    // Tegel 5: Alleen de goedgekeurde blobs (na area+circ filter) — toont effect van die params
    cv::Mat mcBgr = toTile(accepted_.empty() ? mask_clean_ : accepted_);
    cv::putText(mcBgr, "5 ACCEPTED (area+circ)", {8,28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,0}, 2);

    // Tegel 6: Overlay met gedetecteerde sterren
    cv::Mat overlay = toTile(bgr);
    // Schaalfactoren van origineel naar tegel
    float sx = static_cast<float>(tileW) / bgr.cols;
    float sy = static_cast<float>(tileH) / bgr.rows;
    for (const auto& pt : dets) {
        cv::Point tp(static_cast<int>(pt.x * sx), static_cast<int>(pt.y * sy));
        cv::circle(overlay, tp, 10, {0,255,0}, 2);
        cv::circle(overlay, tp,  2, {0,0,255}, -1);
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "6 OVERLAY  n=%d", static_cast<int>(dets.size()));
    cv::putText(overlay, buf, {8,28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,0}, 2);

    // Samenvoegen: rij 1 = t1|valBgr|enhBgr, rij 2 = mfBgr|mcBgr|overlay
    cv::Mat row1, row2, grid;
    cv::hconcat(std::vector<cv::Mat>{t1, valBgr, enhBgr}, row1);
    cv::hconcat(std::vector<cv::Mat>{mfBgr, mcBgr, overlay}, row2);
    cv::vconcat(row1, row2, grid);

    // Stats strip
    cv::Mat strip(65, grid.cols, CV_8UC3, cv::Scalar(26, 26, 26));
    {
        char line1[320], line2[320];
        std::snprintf(line1, sizeof(line1),
            "n=%d  |  contours: %d total   %d rejected_area   %d rejected_circ",
            static_cast<int>(dets.size()),
            n_total_contours_, n_rejected_area_, n_rejected_circ_);
        double appliedThr = std::max(static_cast<double>(cfg_.peakFloor), last_otsu_val_);
        bool clamped = cfg_.peakCeil < 255 && appliedThr > cfg_.peakCeil;
        if (clamped) appliedThr = cfg_.peakCeil;
        std::snprintf(line2, sizeof(line2),
            "detect_on: enhanced (V-bg) %dx%d  |  mask: threshold(enhanced, Otsu=%.0f -> thr=%.0f%s) AND white_mask(S<=%d, V>=%d)  |  centroid_on: raw V",
            enhanced_.cols, enhanced_.rows, last_otsu_val_, appliedThr,
            clamped ? " CLAMPED" : "",
            cfg_.satMax, cfg_.valMin);
        cv::putText(strip, line1, {12, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1);
        cv::putText(strip, line2, {12, 42}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {160, 160, 160}, 1);
    }
    cv::vconcat(grid, strip, grid);
    return grid;
}

// ── Debug-afbeeldingen opslaan ────────────────────────────────────────────────

void StarDetector::saveDebugImages(const cv::Mat& bgr,
                                    const std::vector<cv::Point2f>& detections,
                                    const std::string& prefix) const {
    // 1. Ruwe opname
    cv::imwrite(prefix + "_1_raw.jpg", bgr);

    // 2. HSV Value-kanaal (wat de detector 'ziet')
    if (!hsv_.empty()) {
        std::vector<cv::Mat> ch(3);
        cv::split(hsv_, ch);
        cv::imwrite(prefix + "_2_value.jpg", ch[2]);
        cv::imwrite(prefix + "_3_saturation.jpg", ch[1]);
    }

    // 3. Enhanced (achtergrondafgetrokken)
    if (!enhanced_.empty()) {
        cv::Mat enh8;
        cv::normalize(enhanced_, enh8, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::imwrite(prefix + "_4_enhanced.jpg", enh8);
    }

    // 4. Definitief masker na drempelstelling
    if (!mask_final_.empty())
        cv::imwrite(prefix + "_5_mask_final.jpg", mask_final_);

    // 5. Schoon masker na morfologische opening
    if (!mask_clean_.empty())
        cv::imwrite(prefix + "_6_mask_clean.jpg", mask_clean_);

    // 6. Overlay: gedetecteerde sterren op ruwe opname
    cv::Mat overlay = bgr.clone();
    for (const auto& pt : detections) {
        cv::circle(overlay, cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)),
                   8, cv::Scalar(0, 255, 0), 2);
        cv::circle(overlay, cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)),
                   2, cv::Scalar(0, 0, 255), -1);
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), " %zu sterren", detections.size());
    cv::putText(overlay, buf, {10, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.2,
                cv::Scalar(0, 255, 0), 2);
    cv::imwrite(prefix + "_7_overlay.jpg", overlay);

    std::printf("  Debug: %s_*.jpg opgeslagen\n", prefix.c_str());
}
