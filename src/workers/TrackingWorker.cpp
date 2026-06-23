#include "TrackingWorker.hpp"
#include "worker_util.hpp"

#include "geometry/Transform.hpp"
#include "localiser/Localiser3D.hpp"
#include "tracker/Tracker3D.hpp"
#include "pose/PoseEstimator.hpp"
#include "util/Intrinsics.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <memory>
#include <thread>

// ── File-scope helpers (only called from TrackingWorker::run) ─────────────────

static PoseResult meas_to_pose_result(const CameraPoseMeasurement& meas,
                                       const Intrinsics& intr) {
    PoseResult pose;
    if (!meas.valid) return pose;

    const auto& t = meas.T_world_cam.t;
    pose.x_m       = static_cast<float>(t[0]);
    pose.y_m       = static_cast<float>(t[1]);
    pose.z_m       = static_cast<float>(t[2]);   // raw tracker world z (negative below ceiling)

    // Full ZYX euler from R_world_cam — yaw, pitch, roll all from PnP
    auto eu = euler_from_R(meas.T_world_cam.R);
    pose.yaw_deg   = static_cast<float>(eu.yaw_deg);
    pose.pitch_deg = static_cast<float>(eu.pitch_deg);
    pose.roll_deg  = static_cast<float>(eu.roll_deg);
    pose.theta_deg = pose.yaw_deg;

    pose.n_inliers    = meas.inliers;
    pose.n_detections = meas.detections;
    pose.match_pct    = meas.detections > 0
                      ? 100.f * meas.inliers / (float)meas.detections : 0.f;
    float reproj_px   = static_cast<float>(meas.reprojection_error_px);
    pose.reproj_err_m = reproj_px / std::max(1.f, (float)meas.detections); // rough proxy in m
    pose.scale_est    = static_cast<float>(intr.fx()) / std::max(0.1f, -pose.z_m);  // distance to ceiling stays positive
    pose.pose_confidence = static_cast<float>(meas.confidence);

    if      (pose.match_pct >= 70.f && reproj_px < 3.f)
        pose.verdict = PoseResult::Verdict::Correct;
    else if (pose.match_pct >= 50.f && reproj_px < 5.f)
        pose.verdict = PoseResult::Verdict::Probable;
    else if (pose.match_pct >= 30.f && reproj_px < 8.f)
        pose.verdict = PoseResult::Verdict::Partial;
    else
        pose.verdict = PoseResult::Verdict::Doubtful;

    pose.valid = (pose.verdict != PoseResult::Verdict::Doubtful);
    return pose;
}

static void sanitise_pose_z(PoseResult& pose, float calib_height, float ceiling_height_m) {
    const float dist = -pose.z_m;  // positive distance to ceiling
    if (!std::isfinite(dist) || dist <= 0.0f || dist > ceiling_height_m) {
        pose.z_m = -calib_height;
        pose.pose_confidence = std::min(pose.pose_confidence, 0.69f);
    }
}

static bool pose_inside_map_bounds(const PoseResult& pose, const StarMap& star_map) {
    if (star_map.empty()) return true;

    float min_x = star_map.front().x;
    float max_x = star_map.front().x;
    float min_y = star_map.front().y;
    float max_y = star_map.front().y;
    for (const auto& p : star_map) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }

    constexpr float kMarginM = 0.75f;
    return std::isfinite(pose.x_m) && std::isfinite(pose.y_m)
        && pose.x_m >= min_x - kMarginM && pose.x_m <= max_x + kMarginM
        && pose.y_m >= min_y - kMarginM && pose.y_m <= max_y + kMarginM;
}

// ── Constructor ───────────────────────────────────────────────────────────────

TrackingWorker::TrackingWorker(FrameSlots*         slots,
                               CameraMeas*         cam_meas,
                               VisionPose*         vis_pose,
                               MapState*           map,
                               ControlState*       ctrl,
                               SharedConfig*       config,
                               SharedMemoryServer* blk)
    : m_slots(slots)
    , m_cam_meas(cam_meas)
    , m_vis_pose(vis_pose)
    , m_map(map)
    , m_ctrl(ctrl)
    , m_config(config)
    , m_blk(blk)
{}

// ── run() — Core 2 thread: Localise / Track ───────────────────────────────────

void TrackingWorker::run() {
    pin_to_core(2);  // SCHED_OTHER, affinity only

    // Wait until a star map is loaded.
    while (g_running && !m_map->map_ready.load(std::memory_order_acquire))
        std::this_thread::sleep_for(std::chrono::milliseconds{50});

    if (!g_running) return;

    // Snapshot the generation at construction time.
    // If build_map installs a new index, we detect the change and reconstruct.
    uint32_t seen_generation = m_map->generation.load(std::memory_order_acquire);

    // PnP pipeline objects
    auto make_kalibr_intr = [&]() -> CameraIntrinsics {
        AppConfig cfg_snap;
        {
            std::lock_guard<std::mutex> lk(m_config->mtx);
            cfg_snap = m_config->cfg;
        }
        CameraIntrinsics intr;
        intr.K    = m_ctrl->intr.K.clone();
        // For detections on the already-undistorted frame: use zero distortion.
        intr.dist = cv::Mat::zeros(1, 4, CV_64F);
        intr.resolution = cv::Size(cfg_snap.camera.width, cfg_snap.camera.height);
        return intr;
    };

    AppConfig cfg_snap_init;
    {
        std::lock_guard<std::mutex> lk(m_config->mtx);
        cfg_snap_init = m_config->cfg;
    }
    std::unique_ptr<Localiser3D> localiser3d;
    std::unique_ptr<Tracker3D>   tracker3d;
    if (m_map->map3d_ready.load(std::memory_order_acquire) && m_map->star_index3d) {
        CameraIntrinsics ci = make_kalibr_intr();
        localiser3d = std::make_unique<Localiser3D>(ci, *m_map->star_index3d, cfg_snap_init.localiser);
        tracker3d   = std::make_unique<Tracker3D>  (ci, cfg_snap_init.tracker);
    }

    CameraPoseMeasurement lastMeas3d;        // last CameraPoseMeasurement from PnP
    TrackState state = TrackState::Idle;
    m_ctrl->track_state.store(static_cast<int>(state));

    // Per-slot last-consumed seq. Using a single global last_seq caused every
    // other frame to be dropped: CW alternates slots so both always end up with
    // the same seq value, and the condition s > last_seq failed for whichever
    // slot was written second.
    uint64_t last_seq_per_slot[2] = {0, 0};
    uint64_t bad_since_us = 0;
    bool loss_reported = false;
    constexpr uint64_t kLostTimeoutUs = 1'000'000ULL;

    // Throttled debug logging — print once every kLogEveryN frames
    int  log_frame_counter = 0;
    constexpr int kLogEveryN = 30;

    while (g_running) {
        // If build_map installed a new star index, reconstruct Localiser/Tracker
        // so they no longer hold a reference to the destroyed old StarIndex.
        uint32_t cur_gen = m_map->generation.load(std::memory_order_acquire);
        if (cur_gen != seen_generation) {
            seen_generation = cur_gen;
            // Rebuild PnP objects
            localiser3d.reset();
            tracker3d.reset();
            if (m_map->map3d_ready.load(std::memory_order_acquire) && m_map->star_index3d) {
                AppConfig cfg_snap_gen;
                {
                    std::lock_guard<std::mutex> lk(m_config->mtx);
                    cfg_snap_gen = m_config->cfg;
                }
                CameraIntrinsics ci = make_kalibr_intr();
                localiser3d = std::make_unique<Localiser3D>(ci, *m_map->star_index3d, cfg_snap_gen.localiser);
                tracker3d   = std::make_unique<Tracker3D>  (ci, cfg_snap_gen.tracker);
            }
            state     = TrackState::Localise;
            m_ctrl->track_state.store(static_cast<int>(state));
            bad_since_us = 0;
            loss_reported = false;
            m_blk->block()->tracking_active.store(0, std::memory_order_relaxed);
            m_blk->block()->tracking_lost.store(0, std::memory_order_relaxed);
        }
        // Pick the newest slot that is fresher than what we last consumed from it.
        // Comparing per-slot so both slots are eligible independently — a single
        // global last_seq would drop every second frame because CW alternates evenly,
        // keeping both slots at the same seq value at all times.
        int best = -1;
        uint64_t best_seq = 0;
        for (int i = 0; i < 2; ++i) {
            uint64_t s = (*m_slots)[i].seq.load(std::memory_order_acquire);
            if ((s & 1) == 0 && s > last_seq_per_slot[i]) {
                if (best < 0 || s > best_seq) { best_seq = s; best = i; }
            }
        }
        if (best < 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
            continue;
        }
        // Snapshot the slot contents (seqlock read).
        const FrameSlot& slot = (*m_slots)[best];
        auto detections  = slot.undistorted;
        uint64_t ts_us   = slot.timestamp_us;
        uint64_t capture_us = slot.capture_us;    // Capture timestamp for latency tracking
        // Verify slot wasn't modified mid-read.
        if (slot.seq.load(std::memory_order_acquire) != best_seq) continue;
        last_seq_per_slot[best] = best_seq;

        // Check if a stop command was received.
        if (m_ctrl->track_state.load() == static_cast<int>(TrackState::Idle)) {
            state = TrackState::Idle;
            m_blk->block()->tracking_active.store(0, std::memory_order_relaxed);
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
            continue;
        }
        // start_track resumed us from Idle: local state never got updated because
        // only the Idle branch above synced it. Reset to Localise so the
        // Localise/Tracking branches below actually execute.
        if (state == TrackState::Idle) {
            state = TrackState::Localise;
            bad_since_us = 0;
            loss_reported = false;
        }

        auto hold_bad_frame = [&](const char* reason) {
            uint64_t t = now_us();
            if (bad_since_us == 0)
                bad_since_us = t;

            // Freeze output immediately so Core 3 cannot keep integrating a stale pose.
            m_blk->block()->tracking_active.store(0, std::memory_order_relaxed);

            if (!loss_reported && t - bad_since_us >= kLostTimeoutUs) {
                loss_reported = true;
                m_blk->block()->tracking_lost.store(1, std::memory_order_relaxed);
                std::fprintf(stderr, "[track] Tracking lost: %s\n", reason);
            }
        };

        auto accept_pose = [&](PoseResult pose) {
            sanitise_pose_z(pose, m_ctrl->calib_height.load(std::memory_order_relaxed),
                            m_ctrl->ceiling_height_m.load(std::memory_order_relaxed));
            if (!pose_inside_map_bounds(pose, m_map->star_map)) {
                hold_bad_frame("pose buiten sterkaart");
                return false;
            }

            {
                std::lock_guard<std::mutex> lk(m_vis_pose->mtx);
                m_vis_pose->result = pose;
                ++m_vis_pose->version;
            }
            m_blk->block()->tracking_lost.store(0, std::memory_order_relaxed);
            m_blk->block()->tracking_active.store(1, std::memory_order_relaxed);

            if (m_ctrl->logger.active.load(std::memory_order_relaxed)) {
                const uint64_t vis_mono_us = now_us();
                timespec _vts{}; ::clock_gettime(CLOCK_REALTIME, &_vts);
                const uint64_t vis_real_us = static_cast<uint64_t>(_vts.tv_sec) * 1'000'000ULL
                                           + static_cast<uint64_t>(_vts.tv_nsec) / 1'000ULL;
                const TrackState cur_ts = static_cast<TrackState>(
                    m_ctrl->track_state.load(std::memory_order_relaxed));
                const char* state_str = cur_ts == TrackState::Idle     ? "Idle"     :
                                        cur_ts == TrackState::Localise ? "Localise" : "Tracking";
                const float reproj_px = pose.reproj_err_m
                                      * std::max(1.f, static_cast<float>(pose.n_detections));
                const char* verdict_str =
                    pose.verdict == PoseResult::Verdict::Correct  ? "Correct"  :
                    pose.verdict == PoseResult::Verdict::Probable ? "Probable" :
                    pose.verdict == PoseResult::Verdict::Partial  ? "Partial"  : "Doubtful";
                m_ctrl->logger.row_vision(vis_mono_us, vis_real_us, state_str,
                    pose.x_m, pose.y_m, pose.z_m,
                    pose.roll_deg, pose.pitch_deg, pose.yaw_deg,
                    pose.n_detections, pose.n_inliers,
                    reproj_px, pose.match_pct, verdict_str);
                m_ctrl->logger.record_latency(capture_us, ts_us, vis_mono_us);
            }

            if (loss_reported)
                std::fprintf(stderr, "[track] Tracking recovered\n");
            bad_since_us = 0;
            loss_reported = false;
            return true;
        };

        const int min_stars = std::max(3, m_ctrl->min_tracking_stars.load(std::memory_order_relaxed));
        if (static_cast<int>(detections.size()) < min_stars) {
            state = TrackState::Localise;
            m_ctrl->track_state.store(static_cast<int>(state));
            hold_bad_frame("te weinig sterren zichtbaar");
            continue;
        }

        bool do_log = (++log_frame_counter % kLogEveryN == 0);

        if (!localiser3d || !tracker3d) {
            hold_bad_frame("geen PnP pipeline beschikbaar");
            continue;
        }

        if (state == TrackState::Localise) {
            auto result = localiser3d->localise(detections, ts_us);
            if (do_log) {
                if (result)
                    std::fprintf(stderr,
                        "[PoseDebug] src=Localiser3D dets=%d inliers=%d reproj=%.2fpx\n",
                        result->detections, result->inliers,
                        result->reprojection_error_px);
                else
                    std::fprintf(stderr,
                        "[PoseDebug] src=Localiser3D dets=%d result=null\n",
                        static_cast<int>(detections.size()));
            }
            if (result && result->valid) {
                PoseResult pose = meas_to_pose_result(*result, m_ctrl->intr);
                if (!accept_pose(pose)) {
                    state = TrackState::Localise;
                    m_ctrl->track_state.store(static_cast<int>(state));
                    continue;
                }
                {
                    std::lock_guard<std::mutex> lk(m_cam_meas->mtx);
                    m_cam_meas->meas = *result;
                    m_cam_meas->meas.capture_timestamp_us = capture_us;
                    ++m_cam_meas->version;
                }
                lastMeas3d  = *result;
                state       = TrackState::Tracking;
                m_ctrl->track_state.store(static_cast<int>(state));
            } else {
                hold_bad_frame("geen betrouwbare localisatie (PnP)");
            }

        } else if (state == TrackState::Tracking) {
            auto result = tracker3d->track(lastMeas3d, detections, m_map->star_map3d, ts_us);
            if (do_log && result)
                std::fprintf(stderr,
                    "[PoseDebug] src=Tracker3D dets=%d inliers=%d reproj=%.2fpx reloc=%d\n",
                    result->meas.detections, result->meas.inliers,
                    result->meas.reprojection_error_px,
                    result->relocalise_requested ? 1 : 0);

            if (result && !result->relocalise_requested && result->meas.valid) {
                PoseResult pose = meas_to_pose_result(result->meas, m_ctrl->intr);
                if (!accept_pose(pose)) {
                    state = TrackState::Localise;
                    m_ctrl->track_state.store(static_cast<int>(state));
                    continue;
                }
                {
                    std::lock_guard<std::mutex> lk(m_cam_meas->mtx);
                    m_cam_meas->meas = result->meas;
                    m_cam_meas->meas.capture_timestamp_us = capture_us;
                    ++m_cam_meas->version;
                }
                lastMeas3d  = result->meas;
            } else {
                state = TrackState::Localise;
                m_ctrl->track_state.store(static_cast<int>(state));
                hold_bad_frame(result ? "trackingkwaliteit te laag (PnP)"
                                      : "geen betrouwbare tracking (PnP)");
            }
        }
    }
}
