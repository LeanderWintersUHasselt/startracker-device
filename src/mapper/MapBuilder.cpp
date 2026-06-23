#include "mapper/MapBuilder.hpp"
#include "detector/StarDetector.hpp"
#include "io/CameraReader.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <thread>

MapBuilder::MapBuilder(Intrinsics intr, MapBuilderConfig cfg, DetectorConfig detCfg,
                       CameraConfig camCfg, float calibHeight)
    : intr_(std::move(intr)), cfg_(cfg), detCfg_(detCfg),
      camCfg_(camCfg), calibHeight_(calibHeight) {
    scaleHint_ = intr_.fx() / calibHeight_;
    H_est_ref_to_curr_ = cv::Mat::eye(3, 3, CV_64F);
}

// ── Mutual NN-matching ────────────────────────────────────────────────────────

std::vector<std::pair<int,int>> MapBuilder::mutualNN(
        const std::vector<cv::Point2f>& a,
        const std::vector<cv::Point2f>& b,
        float maxDist) {
    float maxD2 = maxDist * maxDist;
    int na = static_cast<int>(a.size());
    int nb = static_cast<int>(b.size());

    // A → B
    std::vector<int> nn_a2b(na, -1);
    for (int ia = 0; ia < na; ++ia) {
        float best = maxD2; int bestJ = -1;
        for (int ib = 0; ib < nb; ++ib) {
            float dx = b[ib].x - a[ia].x, dy = b[ib].y - a[ia].y;
            float d2 = dx*dx + dy*dy;
            if (d2 < best) { best = d2; bestJ = ib; }
        }
        nn_a2b[ia] = bestJ;
    }

    // B → A
    std::vector<int> nn_b2a(nb, -1);
    for (int ib = 0; ib < nb; ++ib) {
        float best = maxD2; int bestI = -1;
        for (int ia = 0; ia < na; ++ia) {
            float dx = a[ia].x - b[ib].x, dy = a[ia].y - b[ib].y;
            float d2 = dx*dx + dy*dy;
            if (d2 < best) { best = d2; bestI = ia; }
        }
        nn_b2a[ib] = bestI;
    }

    // Mutual: alleen wederzijdse matches
    std::vector<std::pair<int,int>> result;
    for (int ia = 0; ia < na; ++ia) {
        int ib = nn_a2b[ia];
        if (ib >= 0 && nn_b2a[ib] == ia)
            result.emplace_back(ia, ib);
    }
    return result;
}

// ── Frame verwerken ───────────────────────────────────────────────────────────

std::tuple<int,int,int> MapBuilder::processFrame(
        const std::vector<cv::Point2f>& det) {
    int nd = static_cast<int>(det.size());

    // Eerste bruikbaar frame → initialiseer referentie
    if (!initialized_) {
        if (nd < 3) return {0, 0, nd};
        stars_.clear();
        for (const auto& p : det)
            stars_.push_back({p, 1});
        H_est_ref_to_curr_ = cv::Mat::eye(3, 3, CV_64F);
        initialized_ = true;
        std::printf("  Referentie: %d sterren\n", nd);
        return {nd, 0, nd};
    }

    int nm = static_cast<int>(stars_.size());
    if (nd < 3 || nm == 0) return {0, 0, nd};

    // Projecteer kaartsterren naar huidig frame via H_est_ref_to_curr
    std::vector<cv::Point2f> mapRef(nm), mapCurr(nm);
    for (int i = 0; i < nm; ++i) mapRef[i] = stars_[i].pos;
    cv::perspectiveTransform(mapRef, mapCurr, H_est_ref_to_curr_);

    // Mutual NN: detecties vs geprojecteerde kaartsterren
    float matchDist = cfg_.mergeRadiusPx * 4.f;
    auto matches = mutualNN(det, mapCurr, matchDist);

    // Vergroot zoekradius als te weinig matches (H_est drift)
    if (static_cast<int>(matches.size()) < std::max(6, nm / 8))
        matches = mutualNN(det, mapCurr, matchDist * 3.f);

    // Directe similarity fit (RANSAC) als er genoeg matches zijn
    bool goodTransform = false;
    cv::Mat H_curr_to_ref;
    cv::Mat inlierMask;

    if (static_cast<int>(matches.size()) >= 3) {
        std::vector<cv::Point2f> src, dst;
        for (auto& [di, mi] : matches) {
            src.push_back(det[di]);
            dst.push_back(mapRef[mi]);
        }

        cv::Mat A;
        A = cv::estimateAffinePartial2D(src, dst, inlierMask,
                                        cv::RANSAC, 5.0, 2000, 0.99);
        int inliers = inlierMask.empty() ? 0 : cv::countNonZero(inlierMask);

        if (!A.empty() && inliers >= 3) {
            float ratio = static_cast<float>(inliers) / matches.size();
            goodTransform = (ratio >= cfg_.minInlierRatio);
            if (goodTransform) {
                H_curr_to_ref = cv::Mat::eye(3, 3, CV_64F);
                A.copyTo(H_curr_to_ref.rowRange(0, 2));
                H_est_ref_to_curr_ = H_curr_to_ref.inv();
            }
        } else {
            H_curr_to_ref = H_est_ref_to_curr_.inv();
        }
    } else {
        H_curr_to_ref = H_est_ref_to_curr_.inv();
    }

    // A rejected transform must not change the accumulated map.  Previously
    // matches from this frame were still averaged into stars_, which turns a
    // transient mismatch or peripheral residual into persistent map distortion.
    if (!goodTransform)
        return {0, 0, nd};

    // Transformeer ALLE detecties naar referentie-coördinaten
    std::vector<cv::Point2f> detRef;
    cv::perspectiveTransform(det, detRef, H_curr_to_ref);

    // Alleen RANSAC-inliers mogen bestaande kaartsterren bijwerken.  Outlier
    // mutual-NN pairs are known inconsistent with this accepted transform.
    std::vector<bool> isMatched(nd, false);
    std::vector<bool> isOutlierMatch(nd, false);
    std::vector<int>  detToMap(nd, -1);
    for (size_t k = 0; k < matches.size(); ++k) {
        auto [di, mi] = matches[k];
        const bool inlier = !inlierMask.empty()
                         && inlierMask.at<uint8_t>(static_cast<int>(k)) != 0;
        if (inlier) {
            isMatched[di] = true;
            detToMap[di] = mi;
        } else {
            isOutlierMatch[di] = true;
        }
    }

    int n_new = 0, n_matched = 0;
    float mergeR2 = cfg_.mergeRadiusPx * cfg_.mergeRadiusPx;

    for (int di = 0; di < nd; ++di) {
        cv::Point2f ptRef = detRef[di];

        if (isMatched[di]) {
            // Update running average van de bestaande kaartster
            int mi = detToMap[di];
            auto& s = stars_[mi];
            s.pos.x = (s.pos.x * s.count + ptRef.x) / (s.count + 1);
            s.pos.y = (s.pos.y * s.count + ptRef.y) / (s.count + 1);
            ++s.count;
            ++n_matched;
        } else if (!isOutlierMatch[di]) {
            // Zoek dichtstbijzijnde bestaande ster (misschien gemist door matching)
            int bestIdx = -1; float bestD2 = mergeR2;
            for (int si = 0; si < nm; ++si) {
                float dx = ptRef.x - stars_[si].pos.x;
                float dy = ptRef.y - stars_[si].pos.y;
                float d2 = dx*dx + dy*dy;
                if (d2 < bestD2) { bestD2 = d2; bestIdx = si; }
            }

            if (bestIdx >= 0) {
                // Merge met bestaande ster
                auto& s = stars_[bestIdx];
                s.pos.x = (s.pos.x * s.count + ptRef.x) / (s.count + 1);
                s.pos.y = (s.pos.y * s.count + ptRef.y) / (s.count + 1);
                ++s.count;
                ++n_matched;
            } else if (goodTransform && n_new < cfg_.maxNewPerFrame) {
                // Nieuwe ster accepteren (alleen bij betrouwbare transform)
                stars_.push_back({ptRef, 1});
                ++n_new;
            }
        }
    }

    return {n_new, n_matched, nd};
}

// ── Hoogte-kalibratie ─────────────────────────────────────────────────────────
//
// Algoritme:
//   1. Referentieframe: detecteer sterren terwijl opstelling stilstaat
//   2. Gebruiker verschuift opstelling de opgegeven afstand (standaard 30 cm)
//   3. Verschoven frame: detecteer sterren
//   4. Mutual NN-matching tussen beide frames (grote zoekradius)
//   5. estimateAffinePartial2D op gematchte paren → translatie in pixels
//   6. hoogte = fx * calibDistanceM / t_px

MapBuilder::CalibResult MapBuilder::calibrateHeight(
        float calibDistanceM,
        std::function<bool()>                   waitFn,
        std::function<void(const std::string&)> sendFn) {

    // Default waitFn: block on stdin (CLI usage)
    if (!waitFn) {
        waitFn = []() -> bool {
            std::string dummy;
            return static_cast<bool>(std::getline(std::cin, dummy));
        };
    }
    // Default sendFn: print to stdout
    if (!sendFn) {
        sendFn = [](const std::string& msg) { std::printf("%s\n", msg.c_str()); };
    }

    StarDetector detector(detCfg_);
    CameraReader cam(camCfg_);
    std::this_thread::sleep_for(std::chrono::milliseconds{1000}); // camera warmup

    // ── Referentieframe ───────────────────────────────────────────────────────
    sendFn("{\"event\":\"calib_prompt\",\"step\":1,\"msg\":\"Leg de opstelling STIL neer en klik Verder.\"}");
    if (!waitFn()) return {};

    std::vector<cv::Point2f> refDet, shiftDet;
    cv::Mat refFrame;
    {
        std::vector<cv::Point2f> best;
        for (int i = 0; i < 8; ++i) {
            cv::Mat bgr = cam.nextFrame(std::chrono::milliseconds{2000});
            if (bgr.empty()) continue;
            auto det = detector.detect(bgr, intr_);
            if (det.size() > best.size()) { best = det; refFrame = bgr.clone(); }
        }
        refDet = std::move(best);
    }
    if (refDet.size() < 5) {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "{\"event\":\"error\",\"msg\":\"Te weinig sterren in referentieframe (%zu).\"}",
            refDet.size());
        sendFn(buf);
        return {};
    }

    // ── Verschoven frame ──────────────────────────────────────────────────────
    char promptBuf[128];
    std::snprintf(promptBuf, sizeof(promptBuf),
        "{\"event\":\"calib_prompt\",\"step\":2,\"msg\":\"Verschuif de opstelling precies %.0f cm en klik Verder.\"}",
        calibDistanceM * 100.f);
    sendFn(promptBuf);
    if (!waitFn()) return {};

    cv::Mat shiftFrame;
    {
        std::vector<cv::Point2f> best;
        for (int i = 0; i < 8; ++i) {
            cv::Mat bgr = cam.nextFrame(std::chrono::milliseconds{2000});
            if (bgr.empty()) continue;
            auto det = detector.detect(bgr, intr_);
            if (det.size() > best.size()) { best = det; shiftFrame = bgr.clone(); }
        }
        shiftDet = std::move(best);
    }
    if (shiftDet.size() < 5) {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
            "{\"event\":\"error\",\"msg\":\"Te weinig sterren na verschuiving (%zu).\"}",
            shiftDet.size());
        sendFn(buf);
        return {};
    }

    // ── Mutual NN-matching ────────────────────────────────────────────────────
    // Grote zoekradius: 30cm shift ~= 200px bij hoogte 1.5m
    // Gebruik maximaal de halve beeldbreedte als zoekradius
    // cx ≈ halve beeldbreedte → 1.2*cx ≈ 60% van de breedte als zoekradius
    float searchRadius = intr_.cx() * 1.2f;
    auto matches = mutualNN(refDet, shiftDet, searchRadius);

    std::printf("       Matches gevonden: %zu\n", matches.size());

    if (matches.size() < 4) {
        std::printf("[FOUT] Te weinig matches (%zu) tussen ref- en verschoven frame.\n"
                    "       Zorg dat minstens de helft van het beeld overlapt.\n",
                    matches.size());
        return {};
    }

    // Bouw corresponderende puntparen
    std::vector<cv::Point2f> src, dst;
    src.reserve(matches.size());
    dst.reserve(matches.size());
    for (auto& [ri, si] : matches) {
        src.push_back(refDet[ri]);
        dst.push_back(shiftDet[si]);
    }

    // ── Homografie (8 DOF) + decompositie ────────────────────────────────────
    // Identiek aan Python build_starmap.py:
    //   H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
    //   num, Rs, ts, normals = cv2.decomposeHomographyMat(H, K)
    //   scale_factor = calib_distance_m / t_norm
    cv::Mat inlierMask;
    cv::Mat H = cv::findHomography(src, dst, cv::RANSAC, 5.0, inlierMask);

    if (H.empty()) {
        std::printf("[FOUT] Kon geen homografie schatten.\n"
                    "       Zorg voor voldoende overlap.\n");
        return {};
    }

    int inliers = inlierMask.empty() ? 0 : cv::countNonZero(inlierMask);

    // Decomponeer H met camera-matrix K → fysieke translatie t (genormaliseerd op d)
    std::vector<cv::Mat> Rs, ts, normals;
    int numSol = cv::decomposeHomographyMat(H, intr_.K, Rs, ts, normals);

    if (numSol == 0) {
        std::printf("[FOUT] Homografie-decompositie mislukt.\n");
        return {};
    }

    // Beste oplossing: normale wijst naar camera (n_z > 0), grootste t_norm
    int bestIdx = -1;
    double bestNorm = -1.0;
    for (int i = 0; i < numSol; ++i) {
        double nz    = normals[i].at<double>(2);
        double tnorm = cv::norm(ts[i]);
        if (nz > 0 && tnorm > 1e-6 && tnorm > bestNorm) {
            bestNorm = tnorm; bestIdx = i;
        }
    }
    if (bestIdx < 0) { // fallback: grootste t_norm ongeacht normale
        for (int i = 0; i < numSol; ++i) {
            double tnorm = cv::norm(ts[i]);
            if (tnorm > bestNorm) { bestNorm = tnorm; bestIdx = i; }
        }
    }

    if (bestIdx < 0 || bestNorm < 1e-9) {
        std::printf("[FOUT] Nul-translatie — is de camera bewogen?\n");
        return {};
    }

    // scale_factor = calib_distance_m / t_norm  (Python: identiek)
    float height  = static_cast<float>(calibDistanceM / bestNorm);
    float quality = 100.f * static_cast<float>(inliers) / static_cast<float>(matches.size());

    double nz_best = normals[bestIdx].at<double>(2);
    std::printf("    t_norm=%.6f  n_z=%.3f\n", bestNorm, nz_best);

    // ── Match-visualisatie opslaan ───────────────────────────────────────────
    if (!refFrame.empty() && !shiftFrame.empty()) {
        // Naast elkaar: ref (links) + shift (rechts) met lijnen tussen matches
        int W = refFrame.cols, H = refFrame.rows;
        cv::Mat canvas(H, W * 2, CV_8UC3, cv::Scalar(30, 30, 30));
        refFrame.copyTo(canvas(cv::Rect(0, 0, W, H)));
        shiftFrame.copyTo(canvas(cv::Rect(W, 0, W, H)));

        for (size_t i = 0; i < matches.size(); ++i) {
            auto [ri, si] = matches[i];
            cv::Point p1(static_cast<int>(refDet[ri].x),
                         static_cast<int>(refDet[ri].y));
            cv::Point p2(static_cast<int>(shiftDet[si].x) + W,
                         static_cast<int>(shiftDet[si].y));
            bool inl = (inlierMask.rows > 0 &&
                        inlierMask.at<uint8_t>(static_cast<int>(i)));
            cv::Scalar col = inl ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255);
            cv::line(canvas, p1, p2, col, 1);
            cv::circle(canvas, p1, 5, col, 1);
            cv::circle(canvas, p2, 5, col, 1);
        }
        char info[128];
        std::snprintf(info, sizeof(info), "inl=%d/%zu  h=%.3fm",
                      inliers, matches.size(), height);
        cv::putText(canvas, info, {10, 35}, cv::FONT_HERSHEY_SIMPLEX,
                    0.9, cv::Scalar(255,255,0), 2);
        cv::imwrite("/tmp/calib_matches.jpg", canvas);
    }

    std::printf("\n  Inliers    : %d / %zu  (%.0f%%)\n", inliers, matches.size(), quality);
    std::printf("  Hoogte     : %.3f m\n", height);

    if (quality < 50.f)
        std::printf("  [WAARSCHUWING] Lage inlier-ratio — resultaat mogelijk onnauwkeurig.\n");

    CalibResult res;
    res.ok             = true;
    res.height_m       = height;
    res.scale_px_per_m = static_cast<float>(intr_.fx() / height);
    res.t_norm         = static_cast<float>(bestNorm);
    res.n_z            = static_cast<float>(normals[bestIdx].at<double>(2));
    res.inliers        = inliers;
    res.total_matches  = static_cast<int>(matches.size());
    res.calib_dist_m   = calibDistanceM;
    return res;
}

// ── Scan ─────────────────────────────────────────────────────────────────────

StarMap MapBuilder::scan(std::function<bool()> stopFn,
                         std::function<void(const ScanStatus&)> statusFn,
                         std::function<void(const cv::Mat&)> previewFn) {
    std::printf("\nSterkaart scan gestart.\n");
    std::printf("Beweeg de camera langzaam over het plafond.\n");
    if (!stopFn)
        std::printf("Druk Enter om te stoppen.\n\n");
    else
        std::printf("Stoppen gebeurt via de frontend.\n\n");

    std::atomic<bool> stopScan{false};
    std::thread keyThread;
    const bool cliMode = !stopFn;
    if (!stopFn) {
        stopFn = [&stopScan]() { return stopScan.load(); };
        keyThread = std::thread([&stopScan]() {
            std::cin.ignore();
            stopScan.store(true);
        });
    }

    StarDetector detector(detCfg_);

    CameraConfig camCfg = camCfg_;
    camCfg.fps = std::max(cfg_.mapFps, camCfg_.fps);
    CameraReader cam(camCfg);

    cv::Mat remap1, remap2;
    cv::Size remapSize;
    {
        remapSize = cv::Size(camCfg.width, camCfg.height);
        cv::initUndistortRectifyMap(intr_.K, intr_.dist,
                                    cv::noArray(), intr_.K,
                                    remapSize, CV_32FC1,
                                    remap1, remap2);
    }

    using clock = std::chrono::steady_clock;
    auto lastProcess = clock::now();
    double dtTarget  = 1.0 / cfg_.mapFps;

    int totalFrames = 0;

    const std::string WIN_SCAN = "Sterkaart scan";
    if (cliMode) {
        cv::namedWindow(WIN_SCAN, cv::WINDOW_NORMAL);
        cv::resizeWindow(WIN_SCAN, 1280, 720);
    }

    while (!stopFn()) {
        cv::Mat bgr = cam.nextFrame(std::chrono::milliseconds{2000});
        if (bgr.empty()) {
            std::printf("  [WAARSCHUWING] Camera time-out\n");
            continue;
        }

        if (bgr.size() != remapSize) {
            remapSize = bgr.size();
            cv::initUndistortRectifyMap(intr_.K, intr_.dist,
                                        cv::noArray(), intr_.K,
                                        remapSize, CV_32FC1,
                                        remap1, remap2);
        }

        cv::Mat undistorted;
        cv::remap(bgr, undistorted, remap1, remap2, cv::INTER_LINEAR);

        if (previewFn)
            previewFn(undistorted);

        auto now = clock::now();
        double elapsed = std::chrono::duration<double>(now - lastProcess).count();
        if (elapsed < dtTarget) continue;
        lastProcess = now;
        ++totalFrames;

        auto det = detector.detectRawCentroids(undistorted);
        auto [n_new, n_matched, n_det] = processFrame(det);

        std::vector<cv::Point2f> confirmedCurr;
        int confirmed = 0;
        if (initialized_ && !stars_.empty()) {
            std::vector<cv::Point2f> mapPts;
            for (const auto& s : stars_) {
                if (s.count >= cfg_.minFrameCount) {
                    ++confirmed;
                    mapPts.push_back(s.pos);
                }
            }
            if (!mapPts.empty())
                cv::perspectiveTransform(mapPts, confirmedCurr, H_est_ref_to_curr_);
        }

        if (statusFn) {
            ScanStatus st;
            st.frame = totalFrames;
            st.total_stars = static_cast<int>(stars_.size());
            st.confirmed_stars = confirmed;
            st.detected_stars = n_det;
            st.matched_stars = n_matched;
            st.new_stars = n_new;
            st.frame_width = undistorted.cols;
            st.frame_height = undistorted.rows;
            st.detected_points = det;
            st.confirmed_points = confirmedCurr;
            statusFn(st);
        }

        if (cliMode) {
            cv::Mat vis;
            cv::resize(undistorted, vis, {1280, 720});
            float sx = 1280.f / undistorted.cols;
            float sy =  720.f / undistorted.rows;

            for (const auto& p : det) {
                cv::Point tp(static_cast<int>(p.x * sx), static_cast<int>(p.y * sy));
                cv::circle(vis, tp, 8, {0, 200, 255}, 1);
            }

            for (const auto& p : confirmedCurr) {
                cv::Point tp(static_cast<int>(p.x * sx), static_cast<int>(p.y * sy));
                cv::circle(vis, tp, 10, {0, 255, 0}, 2);
                cv::circle(vis, tp, 3, {0, 255, 0}, -1);
            }

            char info[128];
            std::snprintf(info, sizeof(info),
                "frame=%d  kaart=%zu  bevestigd=%d  det=%d  [Enter=stop]",
                totalFrames, stars_.size(), confirmed, n_det);
            cv::putText(vis, info, {8, 30}, cv::FONT_HERSHEY_SIMPLEX,
                        0.7, {255, 255, 0}, 2);
            cv::putText(vis, "GEEL=detectie  GROEN=bevestigd (kaart)", {8, 58},
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, {200, 200, 200}, 1);

            cv::imshow(WIN_SCAN, vis);
            cv::waitKey(1);
        }

        std::printf("\r  frame=%4d  kaart=%3zu ster  bevestigd=%d  det=%2d   ",
                    totalFrames, stars_.size(), confirmed, n_det);
        std::fflush(stdout);
    }

    if (cliMode)
        cv::destroyWindow(WIN_SCAN);

    if (keyThread.joinable())
        keyThread.detach();
    std::printf("\n\nScan gestopt. %zu ruwe sterren in kaart.\n", stars_.size());

    std::vector<Star> kept;
    for (const auto& s : stars_)
        if (s.count >= cfg_.minFrameCount) kept.push_back(s);

    std::printf("Na filter (min %d frames): %zu sterren bewaard.\n",
                cfg_.minFrameCount, kept.size());

    {
        float mergeR2 = cfg_.mergeRadiusPx * cfg_.mergeRadiusPx;
        for (size_t i = 0; i < kept.size(); ) {
            bool merged = false;
            for (size_t j = i + 1; j < kept.size(); ++j) {
                float dx = kept[j].pos.x - kept[i].pos.x;
                float dy = kept[j].pos.y - kept[i].pos.y;
                if (dx*dx + dy*dy < mergeR2) {
                    int total = kept[i].count + kept[j].count;
                    kept[i].pos.x = (kept[i].pos.x * kept[i].count
                                   + kept[j].pos.x * kept[j].count) / total;
                    kept[i].pos.y = (kept[i].pos.y * kept[i].count
                                   + kept[j].pos.y * kept[j].count) / total;
                    kept[i].count = total;
                    kept.erase(kept.begin() + j);
                    merged = true;
                    break;
                }
            }
            if (!merged) ++i;
        }
        std::printf("Na merge  (radius %.0fpx ): %zu sterren uniek.\n",
                    cfg_.mergeRadiusPx, kept.size());
    }

    if (kept.empty()) {
        std::printf("[FOUT] Geen sterren overgebleven na filter.\n");
        return {};
    }

// ── Pixel → meter ─────────────────────────────────────────────────────────
    // Kies centroid van de kaart als oorsprong
    cv::Point2f centroid{0.f, 0.f};
    for (const auto& s : kept) centroid += s.pos;
    centroid *= (1.f / kept.size());

    StarMap result;
    result.reserve(kept.size());
    for (const auto& s : kept) {
        float xm = (s.pos.x - centroid.x) / scaleHint_;
        float ym = (s.pos.y - centroid.y) / scaleHint_;
        result.emplace_back(xm, ym);
    }

    return result;
}
