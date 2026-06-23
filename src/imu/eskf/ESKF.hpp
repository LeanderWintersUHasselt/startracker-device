#pragma once
#include <Eigen/Dense>
#include <array>
#include <cstdint>

// 15-state Error-State Kalman Filter for IMU + vision fusion.
//
// Nominal state: position p (3), velocity v (3), quaternion q (4),
//               accel bias ba (3), gyro bias bg (3)  [total 16 elements]
// Error state:  δp (3), δv (3), δθ (3), δba (3), δbg (3) [15 elements]
//
// World frame: Z-up (gravity = {0,0,-9.81}).
// IMU frame: as calibrated by Kalibr (BNO085 physical frame).
//
// Usage:
//   eskf.init(p0, q0, noise);
//   eskf.predictIMU(accel_m_s2, gyro_rad_s, dt_s, stamp_us);
//   eskf.measureCamera(p_meas, sigma_pos, q_meas, sigma_att,
//                      capture_stamp_us, now_us);
//   auto pose = eskf.getState();

struct EskfNoise {
    double var_acc        = 5.3138e-4;
    double var_omega      = 1.0368e-6;
    double var_acc_bias   = 5.5225e-10;
    double var_omega_bias = 1.0e-12;
    double noise_scale    = 1.0;
    double vel_decay_s    = 0.0;
};

struct EskfState {
    Eigen::Vector3d    position{0, 0, 0};
    Eigen::Vector3d    velocity{0, 0, 0};
    Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
};

struct EskfSnapshot {
    Eigen::Vector3d             p, v, ba, bg;
    Eigen::Quaterniond          q;
    Eigen::Matrix<double,15,15> P;
};

struct ImuHistoryEntry {
    EskfSnapshot    pre_state;  // ESKF state before this IMU step
    Eigen::Vector3d acc_raw;    // raw (not bias-subtracted) accelerometer
    Eigen::Vector3d gyr_raw;    // raw gyroscope
    double          dt_s;
    uint64_t        stamp_us;
};

class ESKF {
public:
    ESKF();

    void init(const Eigen::Vector3d& p0,
              const Eigen::Quaterniond& q0,
              const EskfNoise& noise);

    // IMU propagation — saves snapshot to ring buffer, then predicts.
    void predictIMU(const Eigen::Vector3d& accel_m_s2,
                    const Eigen::Vector3d& gyro_rad_s,
                    double dt_s,
                    uint64_t stamp_us);

    // Camera correction with retroactive compensation.
    // Rolls back to the snapshot at meas_stamp_us, applies position +
    // attitude Kalman updates there, then replays stored IMU steps to now_us.
    // Falls back to direct correction if no matching snapshot exists.
    void measureCamera(const Eigen::Vector3d& p_meas,
                       double sigma_pos,
                       const Eigen::Quaterniond& q_meas,
                       double sigma_att,
                       uint64_t meas_stamp_us,
                       uint64_t now_us);

    EskfState getState() const;
    bool initialised() const { return initialised_; }
    void updateNoise(const EskfNoise& n);

    double velNorm() const { return v_.norm(); }
    double baNorm()  const { return ba_.norm(); }
    double bgNorm()  const { return bg_.norm(); }

private:
    Eigen::Vector3d             p_;
    Eigen::Vector3d             v_;
    Eigen::Quaterniond          q_;
    Eigen::Vector3d             ba_;
    Eigen::Vector3d             bg_;
    Eigen::Matrix<double,15,15> P_;
    EskfNoise                   noise_;
    bool                        initialised_{false};

    static constexpr int kHistSize = 64;  // 640 ms at 100 Hz
    std::array<ImuHistoryEntry, kHistSize> history_;
    int hist_head_{0};
    int hist_count_{0};

    static constexpr double kGravity = 9.81;

    Eigen::Matrix3d R() const { return q_.toRotationMatrix(); }
    static Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    void injectAndReset(const Eigen::Matrix<double,15,1>& dx);

    // IMU prediction math without touching the history buffer.
    // Called by both predictIMU (normal path) and replayForward.
    void predictIMUCore(const Eigen::Vector3d& acc_raw,
                        const Eigen::Vector3d& gyr_raw,
                        double dt_s);

    void applyMeasPos(const Eigen::Vector3d& p_meas, double sigma_m);
    void applyMeasQuat(const Eigen::Quaterniond& q_meas, double sigma_rad);

    EskfSnapshot saveSnapshot() const;
    void restoreSnapshot(const EskfSnapshot& s);

    // Walk ring buffer newest→oldest; return physical index of first entry
    // with stamp_us <= meas_stamp_us, or -1 if none.
    int findSnapshotBefore(uint64_t stamp_us) const;

    // Replay ring buffer entries from start_idx inclusive to until_us.
    // Stops early if a gap > 50 ms is detected between consecutive entries.
    void replayForward(int start_idx, uint64_t until_us);
};
