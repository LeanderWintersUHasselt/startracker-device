#include "IMUWorker.hpp"
#include "worker_util.hpp"

#include "geometry/Calibration.hpp"
#include "util/OneEuroFilter.hpp"
#include "geometry/Transform.hpp"
#include "imu/ImuReader.hpp"
#include "imu/eskf/ESKF.hpp"
#include "ipc/FreeD.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <ctime>
#include <limits>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

IMUWorker::IMUWorker(CameraMeas* cam_meas, VisionPose* vis_pose,
                      ControlState* ctrl, SharedConfig* config,
                      SharedSettings* settings, SharedImuDebug* imu_debug,
                      SharedMemoryServer* blk)
    : m_cam_meas(cam_meas), m_vis_pose(vis_pose), m_ctrl(ctrl),
      m_config(config), m_settings(settings), m_imu_debug(imu_debug),
      m_blk(blk) {}

void IMUWorker::run() {
    pin_to_core(3);  // SCHED_OTHER, affinity only

    auto* blk = m_blk->block();

    // ── IMU initialisation ────────────────────────────────────────────────────
    imu::ImuReader imu_reader("/dev/i2c-1", 0x4A);
    bool imu_ok = imu_reader.start();
    blk->imu_ok.store(imu_ok ? 1 : 0, std::memory_order_relaxed);
    if (!imu_ok) {
        std::fprintf(stderr,
            "[Core3] WARNING: IMU init failed (/dev/i2c-1 addr=0x4A). "
            "FreeD will send with zero orientation.\n");
    }

    // ── Output smoothing (One Euro Filter, one per output channel) ───────────
    OneEuroFilter smooth_roll, smooth_pitch, smooth_yaw;
    OneEuroFilter smooth_x, smooth_y, smooth_z;

    // ── ESKF initialisation ───────────────────────────────────────────────────
    auto eskf_heap = std::make_unique<ESKF>();
    ESKF& eskf = *eskf_heap;
    bool eskf_init_ok = false;
    uint64_t eskf_cam_version = 0;
    uint64_t last_imu_stamp_us = 0;

    const KalibrCalibration cal = make_kalibr_calibration();
    EskfNoise eskf_noise;
    eskf_noise.var_acc        = cal.noise.var_acc;
    eskf_noise.var_omega      = cal.noise.var_omega;
    eskf_noise.var_acc_bias   = cal.noise.var_acc_bias;
    eskf_noise.var_omega_bias = cal.noise.var_omega_bias;
    // noise_scale is set each iteration from a fresh cfg_snap

    auto cv_matx_to_eigen = [](const cv::Matx33d& R) -> Eigen::Matrix3d {
        Eigen::Matrix3d Re;
        Re << R(0,0), R(0,1), R(0,2),
              R(1,0), R(1,1), R(1,2),
              R(2,0), R(2,1), R(2,2);
        return Re;
    };
    auto cv_matx_to_eigen_quat = [&](const cv::Matx33d& R) -> Eigen::Quaterniond {
        return Eigen::Quaterniond(cv_matx_to_eigen(R)).normalized();
    };
    struct EulerDeg { double roll = 0.0, pitch = 0.0, yaw = 0.0; };
    auto eigen_quat_to_euler = [](const Eigen::Quaterniond& q) -> EulerDeg {
        const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        constexpr double rad2deg = 180.0 / M_PI;
        EulerDeg e;
        e.yaw   = std::atan2(R(1, 0), R(0, 0)) * rad2deg;
        e.pitch = std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2))) * rad2deg;
        e.roll  = std::atan2(R(2, 1), R(2, 2)) * rad2deg;
        return e;
    };
    auto eigen_to_cv_matx = [](const Eigen::Matrix3d& R) -> cv::Matx33d {
        return cv::Matx33d(
            R(0,0), R(0,1), R(0,2),
            R(1,0), R(1,1), R(1,2),
            R(2,0), R(2,1), R(2,2));
    };
    auto eigen_R_to_euler = [&](const Eigen::Matrix3d& R) -> EulerDeg {
        return eigen_quat_to_euler(Eigen::Quaterniond(R));
    };
    auto wrap_deg = [](double deg) -> double {
        while (deg >  180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    };
    auto axis_angle_R = [](const std::string& axis, double deg) -> Eigen::Matrix3d {
        const double rad = deg * M_PI / 180.0;
        if (axis == "x") return Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitX()).toRotationMatrix();
        if (axis == "y") return Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitY()).toRotationMatrix();
        if (axis == "z") return Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        return Eigen::Matrix3d::Identity();
    };

    const Eigen::Matrix3d R_cam_imu = cv_matx_to_eigen(cal.T_cam_imu.R);
    const Eigen::Vector3d t_cam_imu{
        cal.T_cam_imu.t[0], cal.T_cam_imu.t[1], cal.T_cam_imu.t[2]};

    struct ImuDebugFrame {
        bool        enabled      = false;
        bool        skip_predict = false;
        std::string perturb_axis = "none";
        double      perturb_deg  = 0.0;
        double vis_roll              = std::numeric_limits<double>::quiet_NaN();
        double vis_pitch             = std::numeric_limits<double>::quiet_NaN();
        double vis_yaw               = std::numeric_limits<double>::quiet_NaN();
        double world_imu_roll        = std::numeric_limits<double>::quiet_NaN();
        double world_imu_pitch       = std::numeric_limits<double>::quiet_NaN();
        double world_imu_yaw         = std::numeric_limits<double>::quiet_NaN();
        double eskf_before_roll      = std::numeric_limits<double>::quiet_NaN();
        double eskf_before_pitch     = std::numeric_limits<double>::quiet_NaN();
        double eskf_before_yaw       = std::numeric_limits<double>::quiet_NaN();
        double meas_roll             = std::numeric_limits<double>::quiet_NaN();
        double meas_pitch            = std::numeric_limits<double>::quiet_NaN();
        double meas_yaw              = std::numeric_limits<double>::quiet_NaN();
        double eskf_after_roll       = std::numeric_limits<double>::quiet_NaN();
        double eskf_after_pitch      = std::numeric_limits<double>::quiet_NaN();
        double eskf_after_yaw        = std::numeric_limits<double>::quiet_NaN();
        double out_roll              = std::numeric_limits<double>::quiet_NaN();
        double out_pitch             = std::numeric_limits<double>::quiet_NaN();
        double out_yaw               = std::numeric_limits<double>::quiet_NaN();
        double roundtrip_yaw_err     = std::numeric_limits<double>::quiet_NaN();
        double ab_current_yaw        = std::numeric_limits<double>::quiet_NaN();
        double ab_inverse_yaw        = std::numeric_limits<double>::quiet_NaN();
    };

    // ── FreeD UDP initialisation ──────────────────────────────────────────────
    std::string init_ip;
    uint16_t    init_port;
    {
        std::lock_guard<std::mutex> lk(m_settings->mtx);
        init_ip   = m_settings->data.freed_ip;
        init_port = static_cast<uint16_t>(m_settings->data.freed_port);
    }
    ipc::FreeD freed(init_ip, init_port);
    ipc::FreeD freed_mirror(init_ip, 40001);  // terminal monitor — run freed_sniff.py on Mac
    blk->freed_connected.store(1, std::memory_order_relaxed);

    // ── Timing ────────────────────────────────────────────────────────────────
    using steady = std::chrono::steady_clock;
    auto t_last = steady::now();

    // Exponential moving averages for Hz metrics
    constexpr double kAlpha = 0.1;
    double freed_hz_ema  = 0.0;
    double vision_hz_ema = 0.0;
    double pose_hz_ema   = 0.0;
    auto t_last_freed  = steady::now();
    auto t_last_vision = steady::now();
    auto t_last_pose   = steady::now();

    // Track last vision pose version to detect new corrections
    uint64_t last_vision_version = 0;
    bool prev_use_imu = false;

    // Cached FreeD target for change detection
    std::string last_freed_ip   = init_ip;
    uint16_t    last_freed_port = init_port;

    // Cached tracker-to-cinema-camera offset (metres, tracker body frame)
    double last_cam_off_x = 0.0;
    double last_cam_off_y = 0.0;
    double last_cam_off_z = 0.0;

    // ── Loop at 100 Hz ────────────────────────────────────────────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        AppConfig cfg_snap;
        {
            std::lock_guard<std::mutex> lk(m_config->mtx);
            cfg_snap = m_config->cfg;
        }

        ImuDebugSettings imu_dbg_settings;
        {
            std::lock_guard<std::mutex> lk(m_imu_debug->mtx);
            imu_dbg_settings = m_imu_debug->data;
        }
        ImuDebugFrame imu_dbg;
        imu_dbg.enabled      = imu_dbg_settings.enabled;
        imu_dbg.skip_predict = imu_dbg.enabled && imu_dbg_settings.skip_predict;
        imu_dbg.perturb_axis = imu_dbg_settings.perturb_axis;
        imu_dbg.perturb_deg  = imu_dbg_settings.perturb_deg;

        Eigen::Matrix3d R_cam_imu_active = R_cam_imu;
        RigidTransform  T_cam_imu_active = cal.T_cam_imu;
        if (imu_dbg.enabled && imu_dbg.perturb_axis != "none" && imu_dbg.perturb_deg != 0.0) {
            R_cam_imu_active  = R_cam_imu * axis_angle_R(imu_dbg.perturb_axis, imu_dbg.perturb_deg);
            T_cam_imu_active.R = eigen_to_cv_matx(R_cam_imu_active);
        }

        auto t_now = steady::now();
        double dt_s = std::chrono::duration<double>(t_now - t_last).count();
        t_last = t_now;

        // Clamp dt to [0.001, 0.05] to avoid instability on startup or stalls
        if (dt_s < 0.001) dt_s = 0.001;
        if (dt_s > 0.050) dt_s = 0.050;

        // Read both clock domains for session logging (before any early-exit paths)
        const uint64_t log_mono_us = now_us();
        timespec _lts{}; ::clock_gettime(CLOCK_REALTIME, &_lts);
        const uint64_t log_real_us = static_cast<uint64_t>(_lts.tv_sec) * 1'000'000ULL
                                   + static_cast<uint64_t>(_lts.tv_nsec) / 1'000ULL;

        // Detect new vision frame for session log
        uint64_t cur_cam_ver;
        {
            std::lock_guard<std::mutex> lk(m_cam_meas->mtx);
            cur_cam_ver = m_cam_meas->version;
        }
        const bool new_vision_frame = (cur_cam_ver != last_vision_version);
        if (new_vision_frame) {
            last_vision_version = cur_cam_ver;
            double dt_v = std::chrono::duration<double>(t_now - t_last_vision).count();
            t_last_vision = t_now;
            if (dt_v > 0.0) {
                vision_hz_ema = (vision_hz_ema == 0.0)
                    ? (1.0 / dt_v)
                    : (kAlpha * (1.0 / dt_v) + (1.0 - kAlpha) * vision_hz_ema);
                blk->vision_hz.store(static_cast<float>(vision_hz_ema), std::memory_order_relaxed);
            }
        }

        const bool imu_only = m_ctrl->imu_only_mode.load(std::memory_order_relaxed);
        if (!blk->tracking_active.load(std::memory_order_relaxed) && !imu_only) {
            if (m_ctrl->logger.active.load(std::memory_order_relaxed)) {
                const TrackState ts = static_cast<TrackState>(
                    m_ctrl->track_state.load(std::memory_order_relaxed));
                m_ctrl->logger.row_pose(log_mono_us, log_real_us,
                    ts == TrackState::Idle ? "Idle" : "Localise",
                    false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            }
            eskf_init_ok = false;
            last_imu_stamp_us = 0;
            smooth_roll.reset(); smooth_pitch.reset(); smooth_yaw.reset();
            smooth_x.reset(); smooth_y.reset(); smooth_z.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
            continue;
        }

        // Check imu_enabled setting (user can toggle in SettingsScreen)
        bool use_imu;
        {
            std::lock_guard<std::mutex> lk(m_settings->mtx);
            use_imu = imu_ok && m_settings->data.imu_enabled;
        }
        blk->imu_ok.store(use_imu ? 1 : 0, std::memory_order_relaxed);
        if (use_imu != prev_use_imu) {
            eskf = ESKF{};
            eskf_init_ok = false;
            eskf_cam_version = 0;
            last_imu_stamp_us = 0;
            prev_use_imu = use_imu;
        }

        // ── ESKF step (gyro/accel prediction + camera correction) ────────────
        RuntimeSettings eskf_rt;
        {
            const uint64_t ts_eskf = now_us();
            if (use_imu) {
                // Read ESKF runtime settings (sigma + vel_decay) under mutex
                {
                    std::lock_guard<std::mutex> lk(m_settings->mtx);
                    eskf_rt = m_settings->data;
                }
                // Apply current noise_scale from live config, then vel_decay_s
                eskf_noise.noise_scale = cfg_snap.eskfNoiseScale;
                EskfNoise eskf_noise_rt = eskf_noise;
                eskf_noise_rt.vel_decay_s = eskf_rt.vel_decay_s;
                eskf.updateNoise(eskf_noise_rt);
                // Initialise on first valid camera measurement, or immediately in
                // IMU-only mode (init at default height).
                if (!eskf_init_ok) {
                    CameraPoseMeasurement cam_snap;
                    uint64_t cam_ver;
                    {
                        std::lock_guard<std::mutex> lk(m_cam_meas->mtx);
                        cam_snap = m_cam_meas->meas;
                        cam_ver  = m_cam_meas->version;
                    }
                    if (cam_snap.valid && cam_ver > 0
                        && imu_reader.readCount() > 0
                        && cam_snap.inliers >= std::max(cfg_snap.eskfMinInitInliers,
                                                        m_ctrl->min_tracking_stars.load())
                        && cam_snap.reprojection_error_px < cfg_snap.eskfMinInitReprojPx) {
                        const RigidTransform T_wi_init =
                            compose(cam_snap.T_world_cam, T_cam_imu_active);
                        Eigen::Vector3d p0{T_wi_init.t[0], T_wi_init.t[1], T_wi_init.t[2]};
                        const Eigen::Quaterniond q_vision_map =
                            cv_matx_to_eigen_quat(T_wi_init.R);
                        eskf.init(p0, q_vision_map, eskf_noise_rt);
                        eskf_cam_version = cam_ver;
                        eskf_init_ok = true;
                    } else if (imu_only) {
                        Eigen::Vector3d p0{0.0, 0.0,
                                           -(double)m_ctrl->calib_height.load()};
                        eskf.init(p0, Eigen::Quaterniond::Identity(), eskf_noise_rt);
                        eskf_init_ok = true;
                    }
                }

                if (eskf_init_ok) {
                    const auto meas = imu_reader.imuMeasurement();
                    if (meas.accel_valid && meas.gyro_valid && meas.stamp_us > 0) {
                        if (last_imu_stamp_us > 0) {
                            const double dt_s =
                                static_cast<double>(meas.stamp_us - last_imu_stamp_us) * 1e-6;
                            if (dt_s > 0.0 && dt_s < 0.5 && !imu_dbg.skip_predict) {
                                // Do not double-integrate vertical acceleration: small gravity/bias
                                // errors show up as visible Z drift. Vision/height remains authoritative
                                // for Z while IMU still propagates X/Y and all angular rates.
                                const Eigen::Vector3d acc{meas.ax, meas.ay, meas.az};
                                const Eigen::Vector3d gyr{meas.gx, meas.gy, meas.gz};
                                eskf.predictIMU(acc, gyr, dt_s, meas.stamp_us);
                            }
                        }
                        last_imu_stamp_us = meas.stamp_us;
                    }

                    // Position correction from latest camera measurement
                    CameraPoseMeasurement cam_snap;
                    uint64_t cam_ver;
                    {
                        std::lock_guard<std::mutex> lk(m_cam_meas->mtx);
                        cam_snap = m_cam_meas->meas;
                        cam_ver  = m_cam_meas->version;
                    }
                    if (cam_ver != eskf_cam_version && cam_snap.valid) {
                        eskf_cam_version = cam_ver;
                        if (ts_eskf >= cam_snap.timestamp_us)
                            m_ctrl->logger.record_eskf_lag(ts_eskf - cam_snap.timestamp_us);
                        // Transform camera pose to IMU frame using Kalibr extrinsic
                        const RigidTransform T_world_imu =
                            compose(cam_snap.T_world_cam, T_cam_imu_active);
                        const Eigen::Vector3d p_meas{
                            T_world_imu.t[0], T_world_imu.t[1], T_world_imu.t[2]};
                        // t_imu = t_cam + timeshift_cam_imu (Kalibr convention)
                        const int64_t shift_us =
                            static_cast<int64_t>(cal.timeshift_cam_imu * 1e6);
                        const uint64_t cap_ts = cam_snap.capture_timestamp_us > 0
                            ? cam_snap.capture_timestamp_us : cam_snap.timestamp_us;
                        const uint64_t meas_stamp = static_cast<uint64_t>(
                            static_cast<int64_t>(cap_ts) + shift_us);
                        // Camera attitude correction: q_world_imu = R_world_cam * R_cam_imu_active
                        const Eigen::Matrix3d R_world_cam = cv_matx_to_eigen(cam_snap.T_world_cam.R);
                        const Eigen::Matrix3d R_world_imu_meas = R_world_cam * R_cam_imu_active;
                        const Eigen::Quaterniond q_cam_att =
                            Eigen::Quaterniond(R_world_imu_meas).normalized();
                        if (imu_dbg.enabled) {
                            const auto eu_vis      = eigen_R_to_euler(R_world_cam);
                            const auto eu_wi       = euler_from_R(T_world_imu.R);
                            const Eigen::Matrix3d R_wic   = R_world_cam * R_cam_imu;
                            const Eigen::Matrix3d R_wiinv  = R_world_cam * R_cam_imu.transpose();
                            const RigidTransform T_roundtrip =
                                compose(T_world_imu, inverse(T_cam_imu_active));
                            const auto eu_roundtrip  = euler_from_R(T_roundtrip.R);
                            const auto eu_ab_current = eigen_R_to_euler(R_wic * R_cam_imu.transpose());
                            const auto eu_ab_inverse = eigen_R_to_euler(R_wiinv * R_cam_imu);
                            const auto eu_meas       = eigen_quat_to_euler(q_cam_att);
                            const auto es_before     = eskf.getState();
                            const auto eu_before     = eigen_quat_to_euler(es_before.attitude);
                            imu_dbg.vis_roll          = eu_vis.roll;
                            imu_dbg.vis_pitch         = eu_vis.pitch;
                            imu_dbg.vis_yaw           = eu_vis.yaw;
                            imu_dbg.world_imu_roll    = eu_wi.roll_deg;
                            imu_dbg.world_imu_pitch   = eu_wi.pitch_deg;
                            imu_dbg.world_imu_yaw     = eu_wi.yaw_deg;
                            imu_dbg.eskf_before_roll  = eu_before.roll;
                            imu_dbg.eskf_before_pitch = eu_before.pitch;
                            imu_dbg.eskf_before_yaw   = eu_before.yaw;
                            imu_dbg.meas_roll         = eu_meas.roll;
                            imu_dbg.meas_pitch        = eu_meas.pitch;
                            imu_dbg.meas_yaw          = eu_meas.yaw;
                            imu_dbg.roundtrip_yaw_err = wrap_deg(eu_roundtrip.yaw_deg - eu_vis.yaw);
                            imu_dbg.ab_current_yaw    = eu_ab_current.yaw;
                            imu_dbg.ab_inverse_yaw    = eu_ab_inverse.yaw;
                        }
                        // Scale sigma by reprojection error: only during fast motion
                        // or blur where reproj exceeds normal (~1.8px). Reference is
                        // 2.5px so stationary tracking gets no inflation.
                        const double reproj_scale = std::max(1.0,
                            cam_snap.reprojection_error_px / 2.5);
                        eskf.measureCamera(p_meas,
                                           eskf_rt.sigma_cam_pos * reproj_scale,
                                           q_cam_att,
                                           eskf_rt.sigma_cam_att * reproj_scale,
                                           meas_stamp, ts_eskf);
                        if (imu_dbg.enabled) {
                            const auto es_after  = eskf.getState();
                            const auto eu_after  = eigen_quat_to_euler(es_after.attitude);
                            const Eigen::Matrix3d R_out_cam =
                                es_after.attitude.toRotationMatrix() * R_cam_imu_active.transpose();
                            const auto eu_out = eigen_R_to_euler(R_out_cam);
                            imu_dbg.eskf_after_roll  = eu_after.roll;
                            imu_dbg.eskf_after_pitch = eu_after.pitch;
                            imu_dbg.eskf_after_yaw   = eu_after.yaw;
                            imu_dbg.out_roll         = eu_out.roll;
                            imu_dbg.out_pitch        = eu_out.pitch;
                            imu_dbg.out_yaw          = eu_out.yaw;
                        }
                    }
                }
            }
        }

        // ── FreeD retarget if settings changed ────────────────────────────────
        {
            std::string  cur_ip;
            uint16_t     cur_port;
            {
                std::lock_guard<std::mutex> lk(m_settings->mtx);
                cur_ip   = m_settings->data.freed_ip;
                cur_port = static_cast<uint16_t>(m_settings->data.freed_port);
                last_cam_off_x = m_settings->data.camera_offset_x_m;
                last_cam_off_y = m_settings->data.camera_offset_y_m;
                last_cam_off_z = m_settings->data.camera_offset_z_m;
            }
            if (cur_ip != last_freed_ip || cur_port != last_freed_port) {
                freed.retarget(cur_ip, cur_port);
                last_freed_ip   = cur_ip;
                last_freed_port = cur_port;
            }
        }

        // ── Compose output pose ─────────────────────────────────────────────────
        double pose_x = 0.0, pose_y = 0.0, pose_z = 0.0;
        double pose_yaw = 0.0;
        PoseResult vision_snap;
        {
            std::lock_guard<std::mutex> lk(m_vis_pose->mtx);
            vision_snap = m_vis_pose->result;
        }

        // ── Tracker world → Render world ─────────────────────────────────────────
        // Tracker world: origin at star-map centre on ceiling, z=0 at ceiling, z<0 below.
        // Render world:  origin on floor directly below, z=0 at floor, z>0 upward.
        // Position: pos_render = pos_tracker + (0, 0, ceiling_height_m)
        // Attitude: ZYX euler maps directly — yaw→FreeD pan, pitch→tilt, roll→roll. No swaps.
        const double ceiling_h = m_ctrl->ceiling_height_m.load(std::memory_order_relaxed);
        double pose_roll = 0.0;
        double pose_pitch = 0.0;
        if (use_imu && eskf.initialised()) {
            auto es = eskf.getState();
            const Eigen::Matrix3d R_world_imu = es.attitude.toRotationMatrix();
            const Eigen::Matrix3d R_world_cam = R_world_imu * R_cam_imu_active.transpose();
            const Eigen::Vector3d p_world_cam = es.position - R_world_cam * t_cam_imu;
            pose_x = p_world_cam(0);
            pose_y = p_world_cam(1);
            pose_z = p_world_cam(2) + ceiling_h;
            auto eu = eigen_quat_to_euler(Eigen::Quaterniond(R_world_cam));
            pose_roll  = eu.roll;
            pose_pitch = eu.pitch;
            pose_yaw   = eu.yaw;
            if (imu_dbg.enabled) {
                imu_dbg.out_roll  = eu.roll;
                imu_dbg.out_pitch = eu.pitch;
                imu_dbg.out_yaw   = eu.yaw;
            }
        } else if (!imu_only && vision_snap.valid) {
            pose_x     = static_cast<double>(vision_snap.x_m);
            pose_y     = static_cast<double>(vision_snap.y_m);
            pose_z     = static_cast<double>(vision_snap.z_m) + ceiling_h;
            pose_yaw   = static_cast<double>(vision_snap.yaw_deg);
            pose_roll  = static_cast<double>(vision_snap.roll_deg);
            pose_pitch = static_cast<double>(vision_snap.pitch_deg);
        }
        if (imu_only) {
            pose_x = 0.0;
            pose_y = 0.0;
            pose_z = static_cast<double>(m_ctrl->calib_height.load());  // already height above floor
        }

        // ── One Euro Filter on output pose ───────────────────────────────────
        {
            const double mc  = static_cast<double>(eskf_rt.smooth_min_cutoff);
            const double bet = static_cast<double>(eskf_rt.smooth_beta);
            constexpr double d2r = M_PI / 180.0;
            constexpr double r2d = 180.0 / M_PI;
            pose_roll  = smooth_roll .filter(pose_roll  * d2r, dt_s, mc, bet, true) * r2d;
            pose_pitch = smooth_pitch.filter(pose_pitch * d2r, dt_s, mc, bet, true) * r2d;
            pose_yaw   = smooth_yaw  .filter(pose_yaw   * d2r, dt_s, mc, bet, true) * r2d;
            pose_x     = smooth_x    .filter(pose_x,           dt_s, mc, bet, false);
            pose_y     = smooth_y    .filter(pose_y,           dt_s, mc, bet, false);
            pose_z     = smooth_z    .filter(pose_z,           dt_s, mc, bet, false);
        }

        // ── Tracker-to-cinema-camera position offset ──────────────────────────
        if (last_cam_off_x != 0.0 || last_cam_off_y != 0.0 || last_cam_off_z != 0.0) {
            constexpr double d2r = M_PI / 180.0;
            const Eigen::Quaterniond q =
                Eigen::AngleAxisd(pose_yaw   * d2r, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pose_pitch * d2r, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(pose_roll  * d2r, Eigen::Vector3d::UnitX());
            const Eigen::Vector3d off_world = q * Eigen::Vector3d(last_cam_off_x, last_cam_off_y, last_cam_off_z);
            pose_x += off_world.x();
            pose_y += off_world.y();
            pose_z += off_world.z();
        }

        // ── Write ESKF pose to shared memory (seqlock) ───────────────────────
        // ── Render camera output frame ────────────────────────────────────────
        // The tracker camera looks UP; the render camera looks FORWARD.
        // This 90° mounting difference means tracker roll = physical forward tilt
        // and tracker pitch = physical left/right roll. Remap here so every
        // downstream consumer (UI, log, FreeD) sees the render camera frame.
        // Yaw is negated: right-handed CCW-positive → CW-positive (FreeD/Unreal).
        const double out_pan  = -pose_yaw;   // CW-positive
        const double out_tilt =  pose_roll;  // physical forward tilt
        const double out_roll =  pose_pitch; // physical left/right roll

        {
            auto t_pose = steady::now();
            uint64_t seq = blk->pose_seq.load(std::memory_order_relaxed);
            blk->pose_seq.store(seq | 1, std::memory_order_release);

            blk->x     = pose_x;
            blk->y     = pose_y;
            blk->z     = pose_z;
            blk->roll  = out_roll;
            blk->pitch = out_tilt;
            blk->yaw   = out_pan;
            blk->pose_confidence = eskf.initialised() ? 1.0 : 0.0;

            blk->pose_timestamp_us =
                static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        t_pose.time_since_epoch()).count());

            blk->pose_seq.store(seq + 2, std::memory_order_release);

            // pose_hz EMA
            double dt_p = std::chrono::duration<double>(t_pose - t_last_pose).count();
            t_last_pose = t_pose;
            if (dt_p > 0.0) {
                pose_hz_ema = (pose_hz_ema == 0.0)
                    ? (1.0 / dt_p)
                    : (kAlpha * (1.0 / dt_p) + (1.0 - kAlpha) * pose_hz_ema);
                blk->pose_hz.store(static_cast<float>(pose_hz_ema), std::memory_order_relaxed);
            }
        }
        // Publish ESKF diagnostic state (atomic, no seqlock needed)
        if (use_imu && eskf.initialised()) {
            blk->eskf_vel_norm.store(static_cast<float>(eskf.velNorm()),
                                     std::memory_order_relaxed);
            blk->eskf_ba_norm.store(static_cast<float>(eskf.baNorm()),
                                    std::memory_order_relaxed);
            blk->eskf_bg_norm.store(static_cast<float>(eskf.bgNorm()),
                                    std::memory_order_relaxed);
        }

        // ── Session log row ───────────────────────────────────────────────────
        if (m_ctrl->logger.active.load(std::memory_order_relaxed)) {
            const TrackState ts = static_cast<TrackState>(
                m_ctrl->track_state.load(std::memory_order_relaxed));
            const char* state_str = ts == TrackState::Idle     ? "Idle"     :
                                    ts == TrackState::Localise ? "Localise" : "Tracking";
            m_ctrl->logger.row_pose(log_mono_us, log_real_us, state_str,
                eskf.initialised(), use_imu,
                pose_x, pose_y, pose_z,
                out_roll, out_tilt, out_pan);
        }

        // ── Send FreeD D1 packet ──────────────────────────────────────────────
        auto t_before_send = steady::now();
        freed.send(out_pan, out_tilt, out_roll, pose_x, pose_y, pose_z);
        freed_mirror.send(out_pan, out_tilt, out_roll, pose_x, pose_y, pose_z);

        // Update freed_hz and freed_latency_ms atomics
        auto t_after_send = steady::now();
        double latency_ms = std::chrono::duration<double, std::milli>(
            t_after_send - t_before_send).count();
        blk->freed_latency_ms.store(
            static_cast<float>(latency_ms), std::memory_order_relaxed);
        {
            const uint64_t send_us = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::microseconds>(
                    t_after_send - t_before_send).count());
            m_ctrl->logger.record_freed_send(send_us);
        }

        double dt_freed = std::chrono::duration<double>(
            t_after_send - t_last_freed).count();
        t_last_freed = t_after_send;
        if (dt_freed > 0.0) {
            freed_hz_ema = (freed_hz_ema == 0.0)
                ? (1.0 / dt_freed)
                : (kAlpha * (1.0 / dt_freed) + (1.0 - kAlpha) * freed_hz_ema);
            blk->freed_hz.store(
                static_cast<float>(freed_hz_ema), std::memory_order_relaxed);
        }

        // ── Sleep for remainder of 10ms budget ────────────────────────────────
        auto t_end = steady::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(
            t_end - t_now).count();
        double sleep_ms = 10.0 - elapsed_ms;
        if (sleep_ms > 0.5) {
            std::this_thread::sleep_for(
                std::chrono::duration<double, std::milli>(sleep_ms));
        }
    }

    // ── Cleanup ───────────────────────────────────────────────────────────────
    imu_reader.stop();
    blk->imu_ok.store(0, std::memory_order_relaxed);
    blk->freed_connected.store(0, std::memory_order_relaxed);
}

// ── runRawImu ─────────────────────────────────────────────────────────────────
// BNO085 quaternion → Euler → FreeD, nothing else.
// No ESKF, no One Euro filter, no camera.  Swap with run() in main.cpp to
// measure the true floor of IMU→FreeD latency.

void IMUWorker::runRawImu() {
    pin_to_core(3);

    imu::ImuReader imu_reader("/dev/i2c-1", 0x4A);
    if (!imu_reader.start()) {
        std::fprintf(stderr, "[runRawImu] IMU init failed — abort\n");
        return;
    }

    std::string freed_ip;
    uint16_t    freed_port;
    {
        std::lock_guard<std::mutex> lk(m_settings->mtx);
        freed_ip   = m_settings->data.freed_ip;
        freed_port = static_cast<uint16_t>(m_settings->data.freed_port);
    }
    ipc::FreeD freed(freed_ip, freed_port);
    std::fprintf(stderr, "[runRawImu] sending to %s:%u\n", freed_ip.c_str(), freed_port);

    using steady = std::chrono::steady_clock;

    while (g_running.load(std::memory_order_relaxed)) {
        auto t0 = steady::now();

        const imu::EulerAngles eu = imu_reader.euler();
        // Axis mapping matches the camera-is-mounted-upward convention in run():
        //   pan  = -yaw  (FreeD expects CW-positive)
        //   tilt =  roll (tracker roll = physical forward tilt)
        //   roll =  pitch
        freed.send(-eu.yaw_deg, eu.roll_deg, eu.pitch_deg, 0.0, 0.0, 0.0);

        double elapsed_ms = std::chrono::duration<double, std::milli>(
            steady::now() - t0).count();
        double sleep_ms = 10.0 - elapsed_ms;
        if (sleep_ms > 0.5)
            std::this_thread::sleep_for(
                std::chrono::duration<double, std::milli>(sleep_ms));
    }

    imu_reader.stop();
}
