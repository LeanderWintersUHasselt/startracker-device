#include "CameraWorker.hpp"

#include "detector/StarDetector.hpp"
#include "detector/StarDetectorLight.hpp"
#include "io/CameraReader.hpp"
#include "ipc.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "worker_util.hpp"

// Camera-specific helper defined in main.cpp
extern void write_preview_fit_from_mat(SharedMemoryServer* shm, const cv::Mat& frame);

CameraWorker::CameraWorker(FrameSlots* slots, ControlState* ctrl,
                            SharedConfig* config, SharedMemoryServer* blk,
                            VisionPose* vis_pose)
    : m_slots(slots), m_ctrl(ctrl), m_config(config), m_blk(blk), m_vis_pose(vis_pose) {}

void CameraWorker::run() {
    pin_to_core(1);

    while (g_running) {
        while (g_running &&
               m_ctrl->track_state.load(std::memory_order_relaxed) == static_cast<int>(TrackState::Idle) &&
               !m_ctrl->cam_pause_req.load(std::memory_order_acquire) &&
               m_ctrl->preview_debug_mode.load(std::memory_order_relaxed) == static_cast<int>(PreviewDebugMode::Off))
            std::this_thread::sleep_for(std::chrono::milliseconds{50});
        if (!g_running) return;

        if (m_ctrl->cam_pause_req.load(std::memory_order_acquire)) {
            m_ctrl->cam_paused.store(true, std::memory_order_release);
            while (m_ctrl->cam_pause_req.load(std::memory_order_acquire) && g_running)
                std::this_thread::sleep_for(std::chrono::milliseconds{50});
            m_ctrl->cam_paused.store(false, std::memory_order_release);
            continue;
        }

        {
            AppConfig cfg_snap;
            {
                std::lock_guard<std::mutex> lk(m_config->mtx);
                cfg_snap = m_config->cfg;
            }

            auto cam = std::make_unique<CameraReader>(cfg_snap.camera);
            std::this_thread::sleep_for(std::chrono::milliseconds{800});

            StarDetector detector_normal(cfg_snap.detector);
            StarDetectorLight detector_light(cfg_snap.detectorLight);
            int cam_frame_counter = 0;


            cv::Mat remap1, remap2;
            {
                cv::Size frame_sz(cfg_snap.camera.width, cfg_snap.camera.height);
                cv::initUndistortRectifyMap(
                    m_ctrl->intr.K, m_ctrl->intr.dist,
                    cv::noArray(), m_ctrl->intr.K,
                    frame_sz, CV_32FC1,
                    remap1, remap2);
            }

            while (g_running &&
                   !m_ctrl->cam_pause_req.load(std::memory_order_acquire) &&
                   (m_ctrl->track_state.load(std::memory_order_relaxed) != static_cast<int>(TrackState::Idle) ||
                    m_ctrl->preview_debug_mode.load(std::memory_order_relaxed) != static_cast<int>(PreviewDebugMode::Off))) {
                cv::Mat bgr = cam->nextFrame(std::chrono::milliseconds{3000});
                if (bgr.empty()) {
                    if (cam->isStopped()) {
                        std::fprintf(stderr, "[Core1] Camera stopped, reopening in 2s...\n");
                        break;
                    }
                    std::fprintf(stderr, "[Core1] Camera timeout\n");
                    continue;
                }

                const uint64_t frame_capture_us = now_us();

                cv::Mat undistorted;
                cv::remap(bgr, undistorted, remap1, remap2, cv::INTER_LINEAR);

                PreviewDebugMode debug_mode = static_cast<PreviewDebugMode>(
                    m_ctrl->preview_debug_mode.load(std::memory_order_relaxed));

                // Normal mode: hoist preview and scale so we can write twice —
                // once before detection (fast image display) and once after
                // (accurate ellipses on the current frame, no FrameSlot delay).
                cv::Mat preview;
                float sx = 0.f, sy = 0.f;
                if (debug_mode == PreviewDebugMode::Off) {
                    cv::resize(undistorted, preview, cv::Size(PREVIEW_W, PREVIEW_H),
                               0, 0, cv::INTER_AREA);
                    sx = static_cast<float>(PREVIEW_W) / static_cast<float>(undistorted.cols);
                    sy = static_cast<float>(PREVIEW_H) / static_cast<float>(undistorted.rows);

                    // Perspective grid using vision-only pose (no ESKF lag).
                    // Straight world lines project to straight image lines — use exactly
                    // 2 sample points per line with analytic horizon clipping.
                    if (m_ctrl->preview_grid.load(std::memory_order_relaxed) &&
                        !m_ctrl->intr.K.empty()) {
                        const double fx_g  = static_cast<double>(m_ctrl->intr.fx());
                        const double fy_g  = static_cast<double>(m_ctrl->intr.fy());
                        const double ppx_g = static_cast<double>(m_ctrl->intr.cx());
                        const double ppy_g = static_cast<double>(m_ctrl->intr.cy());

                        // Gate-filtered vision pose: freeze when stationary (delta < MOVE_GATE),
                        // update directly when moving. Outlier guard rejects single-frame jumps.
                        {
                            PoseResult vis;
                            uint64_t vis_ver;
                            {
                                std::lock_guard<std::mutex> lk(m_vis_pose->mtx);
                                vis     = m_vis_pose->result;
                                vis_ver = m_vis_pose->version;
                            }
                            if (vis.valid && vis_ver != m_grid_vis_ver) {
                                m_grid_vis_ver = vis_ver;
                                m_grid_x = vis.x_m;  // position always follows current pose
                                m_grid_y = vis.y_m;
                                if (!m_grid_init) {
                                    m_grid_yaw   = vis.yaw_deg;
                                    m_grid_pitch = vis.pitch_deg;
                                    m_grid_roll  = vis.roll_deg;
                                    m_grid_init  = true;
                                } else {
                                    auto w180 = [](double d) -> double {
                                        while (d >  180.0) d -= 360.0;
                                        while (d < -180.0) d += 360.0;
                                        return d;
                                    };
                                    const double dy    = w180(vis.yaw_deg   - m_grid_yaw);
                                    const double dp    = w180(vis.pitch_deg - m_grid_pitch);
                                    const double dr    = w180(vis.roll_deg  - m_grid_roll);
                                    const double delta = std::sqrt(dy*dy + dp*dp + dr*dr);

                                    constexpr double MAX_JUMP   = 12.0; // outlier threshold (°)
                                    constexpr double MOVE_GATE  =  0.5; // dead-zone radius (°)
                                    constexpr double STOP_GATE  =  0.2; // hysteresis to stop (°)
                                    constexpr int    STOP_FRAMES =   4; // frames below stop gate

                                    if (delta < MAX_JUMP) {
                                        if (delta > MOVE_GATE) {
                                            // Moving — snap directly, reset still counter
                                            m_grid_yaw   = vis.yaw_deg;
                                            m_grid_pitch = vis.pitch_deg;
                                            m_grid_roll  = vis.roll_deg;
                                            m_grid_moving      = true;
                                            m_grid_still_frames = 0;
                                        } else if (m_grid_moving) {
                                            // Was moving, now slowing — keep updating until settled
                                            m_grid_yaw   = vis.yaw_deg;
                                            m_grid_pitch = vis.pitch_deg;
                                            m_grid_roll  = vis.roll_deg;
                                            if (delta < STOP_GATE) {
                                                if (++m_grid_still_frames >= STOP_FRAMES)
                                                    m_grid_moving = false;
                                            } else {
                                                m_grid_still_frames = 0;
                                            }
                                        }
                                        // else: stationary and within gate — freeze
                                    }
                                    // else: outlier — freeze
                                }
                            }
                        }
                        double yw, pw, rw;
                        if (m_grid_init) {
                            yw = m_grid_yaw   * CV_PI / 180.0;
                            pw = m_grid_pitch * CV_PI / 180.0;
                            rw = m_grid_roll  * CV_PI / 180.0;
                        } else {
                            // No vision yet — fall back to ESKF pose
                            const SharedBlock* blk = m_blk->block();
                            yw = -blk->yaw   * CV_PI / 180.0;
                            pw =  blk->roll  * CV_PI / 180.0;
                            rw =  blk->pitch * CV_PI / 180.0;
                        }

                        // R_world_cam = Rz(yw)*Ry(pw)*Rx(rw)  (row-major)
                        const double cyw = std::cos(yw), syw = std::sin(yw);
                        const double cpw = std::cos(pw), spw = std::sin(pw);
                        const double crw = std::cos(rw), srw = std::sin(rw);
                        const double Rwc[3][3] = {
                            { cyw*cpw,  cyw*spw*srw - syw*crw,  cyw*spw*crw + syw*srw },
                            { syw*cpw,  syw*spw*srw + cyw*crw,  syw*spw*crw - cyw*srw },
                            { -spw,     cpw*srw,                 cpw*crw               }
                        };

                        const cv::Rect img_rect(0, 0, PREVIEW_W, PREVIEW_H);

                        // Draw a world-space line Base + t*Dir as a straight image line.
                        // Clips analytically at the camera's z=0 horizon — no samples needed.
                        auto draw_world_line = [&](double bx, double by, double bz,
                                                   double dx, double dy, double dz,
                                                   const cv::Scalar& col) {
                            // Camera-z along the line: pcz(t) = A*t + B
                            const double A = Rwc[0][2]*dx + Rwc[1][2]*dy + Rwc[2][2]*dz;
                            const double B = Rwc[0][2]*bx + Rwc[1][2]*by + Rwc[2][2]*bz;
                            constexpr double EPS  = 0.08;
                            constexpr double TMAX = 10.0;
                            double t_lo = -TMAX, t_hi = TMAX;
                            if (std::abs(A) > 1e-9) {
                                const double tc = (EPS - B) / A;
                                if (A > 0) t_lo = std::max(t_lo, tc);
                                else       t_hi = std::min(t_hi, tc);
                            } else if (B <= EPS) return;
                            if (t_lo >= t_hi) return;

                            auto proj = [&](double t) -> cv::Point {
                                const double wx = bx + t*dx, wy = by + t*dy, wz = bz + t*dz;
                                const double pcx = Rwc[0][0]*wx + Rwc[1][0]*wy + Rwc[2][0]*wz;
                                const double pcy = Rwc[0][1]*wx + Rwc[1][1]*wy + Rwc[2][1]*wz;
                                const double pcz = A*t + B;
                                const int u = static_cast<int>((fx_g*pcx/pcz + ppx_g)*sx + 0.5);
                                const int v = static_cast<int>((fy_g*pcy/pcz + ppy_g)*sy + 0.5);
                                return { std::clamp(u, -32000, 32000),
                                         std::clamp(v, -32000, 32000) };
                            };

                            cv::Point p1 = proj(t_lo), p2 = proj(t_hi);
                            if (cv::clipLine(img_rect, p1, p2))
                                cv::line(preview, p1, p2, col, 2, cv::LINE_AA);
                        };

                        // Rolling perspective grid: same tan(30°) spacing as original,
                        // but centred on camera so lines never disappear at map edges.
                        const double h = std::max(0.1, static_cast<double>(cfg_snap.calibHeight));
                        const double cam_xn = m_grid_x / h;
                        const double cam_yn = m_grid_y / h;

                        const double sn = std::tan(30.0 * CV_PI / 180.0);  // 0.577 — original spacing
                        const int ix = static_cast<int>(std::round(cam_xn / sn));
                        const int iy = static_cast<int>(std::round(cam_yn / sn));
                        constexpr int N = 6;  // lines each side — always covers the FoV

                        // Green: constant-Y lines
                        for (int n = iy - N; n <= iy + N; ++n)
                            draw_world_line(-cam_xn, n * sn - cam_yn, 1.0,
                                            1, 0, 0,  cv::Scalar(0, 200, 0));

                        // Red: constant-X lines
                        for (int n = ix - N; n <= ix + N; ++n)
                            draw_world_line(n * sn - cam_xn, -cam_yn, 1.0,
                                            0, 1, 0,  cv::Scalar(0, 0, 210));
                    }

                    // First write: previous-frame ellipses on a temp copy — no blank flash.
                    // Keep `preview` clean so the second write can overlay fresh ellipses.
                    {
                        cv::Mat first = preview.clone();
                        for (const auto& el : detector_light.lastEllipses()) {
                            cv::RotatedRect rr(
                                cv::Point2f(el.center.x * sx, el.center.y * sy),
                                cv::Size2f(el.size.width * sx, el.size.height * sy),
                                el.angle);
                            cv::ellipse(first, rr, cv::Scalar(0, 220, 0), cv::FILLED);
                        }
                        m_blk->writePreview(first.data, PREVIEW_W, PREVIEW_H);
                    }
                }

                std::vector<cv::Point2f> detections = cfg_snap.useHeavyDetector
                    ? detector_normal.detectRawCentroids(undistorted)
                    : detector_light.detectRawCentroids(undistorted);

                if (debug_mode == PreviewDebugMode::Normal) {
                    auto vis = cfg_snap.useHeavyDetector
                        ? detections
                        : detector_normal.detectRawCentroids(undistorted);
                    cv::Mat grid = detector_normal.makeDebugGrid(undistorted, vis, 640, 360);
                    write_preview_fit_from_mat(m_blk, grid);
                } else if (debug_mode == PreviewDebugMode::Light) {
                    auto vis = cfg_snap.useHeavyDetector
                        ? detector_light.detectRawCentroids(undistorted)
                        : detections;
                    cv::Mat grid = detector_light.makeDebugGrid(undistorted, vis, 640, 360);
                    write_preview_fit_from_mat(m_blk, grid);
                } else if (!preview.empty()) {
                    // Second write: current-frame ellipses on the clean base image
                    for (const auto& el : detector_light.lastEllipses()) {
                        cv::RotatedRect rr(
                            cv::Point2f(el.center.x * sx, el.center.y * sy),
                            cv::Size2f(el.size.width * sx, el.size.height * sy),
                            el.angle);
                        cv::ellipse(preview, rr, cv::Scalar(0, 220, 0), cv::FILLED);
                    }
                    m_blk->writePreview(preview.data, PREVIEW_W, PREVIEW_H);
                }

                if (++cam_frame_counter % 30 == 0)
                    std::fprintf(stderr, "[PoseDebug] Core1 dets=%d\n",
                                 static_cast<int>(detections.size()));

                int slot_idx = ((*m_slots)[0].seq.load(std::memory_order_relaxed) <=
                                (*m_slots)[1].seq.load(std::memory_order_relaxed)) ? 0 : 1;
                FrameSlot& slot = (*m_slots)[slot_idx];
                uint64_t s = slot.seq.load(std::memory_order_relaxed) & ~uint64_t(1);
                slot.seq.store(s | 1, std::memory_order_release);
                slot.undistorted  = detections;
                slot.frame        = undistorted;
                // Subtract ISP pipeline delay so ESKF rolls back to actual sensor
                // readout time, not pipe-arrival time. --buffer-count 2 means the
                // sensor captured this frame ~2 frame-periods before it arrived here.
                const uint64_t isp_delay_us = 1u * (1000000u / static_cast<uint64_t>(cfg_snap.camera.fps));
                slot.capture_us   = (frame_capture_us > isp_delay_us)
                    ? frame_capture_us - isp_delay_us
                    : frame_capture_us;
                slot.timestamp_us = now_us();
                slot.seq.store(s + 2, std::memory_order_release);
            }
        }

        if (m_ctrl->cam_pause_req.load(std::memory_order_acquire)) {
            m_ctrl->cam_paused.store(true, std::memory_order_release);
            while (m_ctrl->cam_pause_req.load(std::memory_order_acquire) && g_running)
                std::this_thread::sleep_for(std::chrono::milliseconds{50});
            m_ctrl->cam_paused.store(false, std::memory_order_release);
        } else if (g_running) {
            std::this_thread::sleep_for(std::chrono::seconds{2});
        }
    }
}
