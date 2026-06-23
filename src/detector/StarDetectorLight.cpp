#include "detector/StarDetectorLight.hpp"
#include <cmath>
#include <cstdio>

StarDetectorLight::StarDetectorLight(DetectorLightConfig cfg) : cfg_(cfg) {}

// ── detectRawCentroids ────────────────────────────────────────────────────────
// BGR → grijs → downsample → blur → Otsu → contouren (area + circulariteit)
// Returns pixel coords in the INPUT image space — no distortion correction.

std::vector<cv::Point2f> StarDetectorLight::detectRawCentroids(const cv::Mat& bgr) const {
    int ds = std::max(1, cfg_.downsample);

    // 1. Grijs
    cv::cvtColor(bgr, gray_, cv::COLOR_BGR2GRAY);

    // 2. Downsample
    if (ds > 1)
        cv::resize(gray_, small_, {gray_.cols / ds, gray_.rows / ds},
                   0, 0, cv::INTER_AREA);
    else
        small_ = gray_;

    // 3. Blur
    int bk = cfg_.blurKernel;
    if (bk > 1) {
        if (bk % 2 == 0) ++bk;
        cv::GaussianBlur(small_, blurred_, {bk, bk}, 0);
    } else {
        blurred_ = small_;
    }

    // 4. Otsu of vaste drempel — altijd Otsu berekenen voor display
    {
        cv::Mat tmp;
        last_otsu_val_ = cv::threshold(blurred_, tmp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    }
    if (cfg_.threshold == 0)
        cv::threshold(blurred_, thresh_, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    else
        cv::threshold(blurred_, thresh_, cfg_.threshold, 255, cv::THRESH_BINARY);

    // 4b. Morfologische closing: vult kleine gaten/inkepingen in de blob zodat
    // gedeeltelijk belichte markers (rand van beeld) meer cirkelvormig worden.
    if (cfg_.morphClose > 0) {
        int mk = cfg_.morphClose % 2 == 0 ? cfg_.morphClose + 1 : cfg_.morphClose;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {mk, mk});
        cv::morphologyEx(thresh_, thresh_, cv::MORPH_CLOSE, kernel);
    }

    // 5. Contouren + filter
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point2f> centroids;
    ellipses_.clear();
    contour_results_.clear();
    n_total_contours_ = 0;
    n_rejected_area_  = 0;
    n_rejected_circ_  = 0;
    accepted_ = cv::Mat::zeros(gray_.size(), CV_8U);

    for (const auto& cnt : contours) {
        ++n_total_contours_;

        // Build absolute full-res contour points for contour_results_ and accepted_
        std::vector<cv::Point> cnt_abs_fr;
        cnt_abs_fr.reserve(cnt.size());
        for (const auto& p : cnt)
            cnt_abs_fr.push_back({p.x * ds, p.y * ds});

        float area         = static_cast<float>(cv::contourArea(cnt));
        float area_fullres = area * static_cast<float>(ds * ds);
        if (area_fullres < cfg_.areaMin || area_fullres > cfg_.areaMax) {
            ++n_rejected_area_;
            contour_results_.push_back({std::move(cnt_abs_fr), ContourResult::Verdict::RejectedArea});
            continue;
        }

        float perim = static_cast<float>(cv::arcLength(cnt, true));
        if (perim < 1e-3f) continue;  // degenerate, not counted as a filter rejection

        float circ = 4.f * static_cast<float>(CV_PI) * area / (perim * perim);
        if (circ < cfg_.circMin) {
            ++n_rejected_circ_;
            contour_results_.push_back({std::move(cnt_abs_fr), ContourResult::Verdict::RejectedCirc});
            continue;
        }

        // Intensity-weighted centroid on full-res gray_ (no downscale bias).
        cv::Rect bbox_ds = cv::boundingRect(cnt);
        cv::Rect bbox_fr = cv::Rect(bbox_ds.x * ds, bbox_ds.y * ds,
                                     bbox_ds.width * ds, bbox_ds.height * ds)
                           & cv::Rect(0, 0, gray_.cols, gray_.rows);
        if (bbox_fr.empty()) continue;

        cv::Mat blob_mask = cv::Mat::zeros(bbox_fr.size(), CV_8U);
        std::vector<cv::Point> cnt_fr;
        cnt_fr.reserve(cnt.size());
        for (const auto& p : cnt)
            cnt_fr.push_back(cv::Point(p.x * ds - bbox_fr.x, p.y * ds - bbox_fr.y));
        cv::fillPoly(blob_mask, std::vector<std::vector<cv::Point>>{cnt_fr}, 255);

        cv::Mat roi = cv::Mat::zeros(bbox_fr.size(), CV_8U);
        gray_(bbox_fr).copyTo(roi, blob_mask);
        cv::Moments m = cv::moments(roi, false);
        if (m.m00 < 1e-6) continue;

        cv::Point2f centre(
            static_cast<float>(m.m10 / m.m00) + bbox_fr.x,
            static_cast<float>(m.m01 / m.m00) + bbox_fr.y
        );

        // Record accepted contour and fill accepted_ mask
        contour_results_.push_back({cnt_abs_fr, ContourResult::Verdict::Accepted});
        cv::fillPoly(accepted_,
                     std::vector<std::vector<cv::Point>>{cnt_abs_fr}, 255);

        // Ellipse used only for preview overlay
        cv::RotatedRect ellipse_fr;
        if (static_cast<int>(cnt.size()) >= 5) {
            cv::RotatedRect e_ds = cv::fitEllipse(cnt);
            ellipse_fr = cv::RotatedRect(
                centre,
                cv::Size2f(e_ds.size.width * ds, e_ds.size.height * ds),
                e_ds.angle);
        } else {
            ellipse_fr = cv::RotatedRect(centre,
                cv::Size2f(float(bbox_ds.width * ds), float(bbox_ds.height * ds)), 0.f);
        }
        ellipses_.push_back(ellipse_fr);
        centroids.emplace_back(centre.x, centre.y);
    }

    return centroids;
}

// ── detectUndistortedCentroids ────────────────────────────────────────────────
// detectRawCentroids + cv::undistortPoints(K, dist).
// Use on a raw (distorted) image when working in undistorted pixel space.

std::vector<cv::Point2f> StarDetectorLight::detectUndistortedCentroids(
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

std::vector<cv::Point2f> StarDetectorLight::detect(const cv::Mat& bgr,
                                                    const Intrinsics& intr) const {
    return detectUndistortedCentroids(bgr, intr.K, intr.dist);
}

// ── makeDebugGrid ─────────────────────────────────────────────────────────────

cv::Mat StarDetectorLight::makeDebugGrid(const cv::Mat& bgr,
                                          const std::vector<cv::Point2f>& dets,
                                          int tileW, int tileH) const {
    cv::Size tile(tileW, tileH);

    auto toTile = [&](const cv::Mat& src, const char* label, const cv::Scalar& color) -> cv::Mat {
        cv::Mat tmp, out;
        if (src.empty()) {
            out = cv::Mat(tile, CV_8UC3, cv::Scalar(40, 40, 40));
        } else {
            tmp = src;
            if (tmp.depth() != CV_8U)
                cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8U);
            if (tmp.channels() == 1)
                cv::cvtColor(tmp, out, cv::COLOR_GRAY2BGR);
            else
                out = tmp.clone();
            cv::resize(out, out, tile);
        }
        cv::putText(out, label, {8, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
        return out;
    };

    // Panel 1: RAW
    cv::Mat t1 = toTile(bgr, "1 RAW", {0, 255, 255});

    // Panel 2: GRAY
    cv::Mat t2 = toTile(gray_, "2 GRAY", {0, 255, 255});

    // Panel 3: THRESHOLD
    cv::Mat t3 = toTile(thresh_, "3 THRESHOLD", {0, 255, 0});

    // Panel 4: ALL CONTOURS — outlines on gray, colour-coded by filter verdict
    cv::Mat t4;
    {
        cv::Mat base;
        cv::cvtColor(gray_, base, cv::COLOR_GRAY2BGR);
        cv::resize(base, t4, tile);
        const float sx = static_cast<float>(tileW) / static_cast<float>(gray_.cols);
        const float sy = static_cast<float>(tileH) / static_cast<float>(gray_.rows);
        for (const auto& cr : contour_results_) {
            std::vector<cv::Point> pts;
            pts.reserve(cr.cnt_fr.size());
            for (const auto& p : cr.cnt_fr)
                pts.push_back({static_cast<int>(p.x * sx), static_cast<int>(p.y * sy)});
            cv::Scalar color =
                cr.verdict == ContourResult::Verdict::Accepted     ? cv::Scalar(0, 255,   0) :
                cr.verdict == ContourResult::Verdict::RejectedArea ? cv::Scalar(0,   0, 255) :
                                                                      cv::Scalar(255,  0,   0);
            cv::polylines(t4, std::vector<std::vector<cv::Point>>{pts}, true, color, 1);
        }
        cv::putText(t4, "4 ALL CONTOURS  G=ok R=area B=circ", {8, 28},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 255}, 1);
    }

    // Panel 5: ACCEPTED
    cv::Mat t5 = toTile(accepted_, "5 ACCEPTED", {0, 255, 0});

    // Panel 6: OVERLAY
    cv::Mat overlay = toTile(bgr, "", {0, 255, 0});
    {
        float sx = static_cast<float>(tileW) / bgr.cols;
        float sy = static_cast<float>(tileH) / bgr.rows;
        for (const auto& pt : dets) {
            cv::Point tp(static_cast<int>(pt.x * sx), static_cast<int>(pt.y * sy));
            cv::circle(overlay, tp, 10, {0, 255, 0}, 2);
            cv::circle(overlay, tp,  2, {0, 0, 255}, -1);
        }
        char buf[64];
        std::snprintf(buf, sizeof(buf), "6 OVERLAY  n=%d", static_cast<int>(dets.size()));
        cv::putText(overlay, buf, {8, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
    }

    // Assemble grid
    cv::Mat row1, row2, grid;
    cv::hconcat(std::vector<cv::Mat>{t1, t2, t3}, row1);
    cv::hconcat(std::vector<cv::Mat>{t4, t5, overlay}, row2);
    cv::vconcat(row1, row2, grid);

    // Stats strip
    cv::Mat strip(65, grid.cols, CV_8UC3, cv::Scalar(26, 26, 26));
    {
        char line1[320], line2[320], thr_str[64];
        if (cfg_.threshold == 0)
            std::snprintf(thr_str, sizeof(thr_str), "Otsu(%.0f)", last_otsu_val_);
        else
            std::snprintf(thr_str, sizeof(thr_str), "%d (Otsu=%.0f)", cfg_.threshold, last_otsu_val_);
        std::snprintf(line1, sizeof(line1),
            "n=%d  |  contours: %d total   %d rejected_area   %d rejected_circ",
            static_cast<int>(dets.size()),
            n_total_contours_, n_rejected_area_, n_rejected_circ_);
        std::snprintf(line2, sizeof(line2),
            "detect_on: gray %dx%d  |  mask: threshold(gray, thr=%s)  |  centroid_on: gray",
            gray_.cols, gray_.rows, thr_str);
        cv::putText(strip, line1, {12, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 1);
        cv::putText(strip, line2, {12, 42}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {160, 160, 160}, 1);
    }
    cv::vconcat(grid, strip, grid);
    return grid;
}
