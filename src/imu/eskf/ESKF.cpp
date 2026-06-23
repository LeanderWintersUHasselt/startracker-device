#include "imu/eskf/ESKF.hpp"
#include <cmath>

ESKF::ESKF() {
    p_.setZero();
    v_.setZero();
    q_ = Eigen::Quaterniond::Identity();
    ba_.setZero();
    bg_.setZero();
    P_.setZero();
    hist_head_  = 0;
    hist_count_ = 0;
}

void ESKF::init(const Eigen::Vector3d& p0,
                const Eigen::Quaterniond& q0,
                const EskfNoise& noise) {
    p_  = p0;
    v_.setZero();
    q_  = q0.normalized();
    ba_.setZero();
    bg_.setZero();
    noise_ = noise;

    P_.setZero();
    P_.block<3,3>(0,0)   = Eigen::Matrix3d::Identity() * 0.01;   // position  σ=0.1m
    P_.block<3,3>(3,3)   = Eigen::Matrix3d::Identity() * 0.01;   // velocity  σ=0.1m/s
    P_.block<3,3>(6,6)   = Eigen::Matrix3d::Identity() * 0.001;  // attitude  σ=0.032rad
    P_.block<3,3>(9,9)   = Eigen::Matrix3d::Identity() * 1e-4;   // acc bias
    P_.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * 1e-6;   // gyro bias

    hist_head_  = 0;
    hist_count_ = 0;
    initialised_ = true;
}

// ── Helpers ───────────────────────────────────────────────────────────────────

Eigen::Matrix3d ESKF::skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return S;
}

// ── predictIMUCore ────────────────────────────────────────────────────────────
// Existing IMU prediction math, extracted verbatim from the old predictIMU.
// Does NOT touch the history buffer — safe to call during replay.

void ESKF::predictIMUCore(const Eigen::Vector3d& accel,
                           const Eigen::Vector3d& gyro,
                           double dt) {
    if (!initialised_ || dt <= 0.0) return;
    Eigen::Vector3d a = accel - ba_;
    Eigen::Vector3d w = gyro  - bg_;
    Eigen::Matrix3d Rw = R();
    const Eigen::Vector3d g_w{0.0, 0.0, -kGravity};

    Eigen::Vector3d p_new = p_ + v_ * dt + 0.5 * (Rw * a + g_w) * dt * dt;
    Eigen::Vector3d v_new = v_ + (Rw * a + g_w) * dt;

    if (noise_.vel_decay_s > 0.0)
        v_new *= std::exp(-dt / noise_.vel_decay_s);

    double wn = w.norm();
    Eigen::Quaterniond dq;
    if (wn > 1e-8) {
        double half_angle = 0.5 * wn * dt;
        dq.w() = std::cos(half_angle);
        dq.vec() = (std::sin(half_angle) / wn) * w;
    } else {
        dq = Eigen::Quaterniond{1.0, 0.5*w(0)*dt, 0.5*w(1)*dt, 0.5*w(2)*dt};
        dq.normalize();
    }
    Eigen::Quaterniond q_new = (q_ * dq).normalized();

    Eigen::Matrix<double,15,15> F = Eigen::Matrix<double,15,15>::Identity();
    F.block<3,3>(0,3)  = Eigen::Matrix3d::Identity() * dt;
    F.block<3,3>(3,6)  = -Rw * skew(a) * dt;
    F.block<3,3>(3,9)  = -Rw * dt;
    F.block<3,3>(6,12) = -Eigen::Matrix3d::Identity() * dt;

    double va  = noise_.var_acc   * noise_.noise_scale;
    double vo  = noise_.var_omega * noise_.noise_scale;
    double vab = noise_.var_acc_bias;
    double vob = noise_.var_omega_bias;

    Eigen::Matrix<double,15,15> Q = Eigen::Matrix<double,15,15>::Zero();
    Q.block<3,3>(3,3)   = Rw * (Eigen::Matrix3d::Identity() * va * dt * dt) * Rw.transpose();
    Q.block<3,3>(6,6)   = Eigen::Matrix3d::Identity() * vo * dt * dt;
    Q.block<3,3>(9,9)   = Eigen::Matrix3d::Identity() * vab * dt;
    Q.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * vob * dt;

    P_ = F * P_ * F.transpose() + Q;
    P_ = 0.5 * (P_ + P_.transpose());

    p_ = p_new;
    v_ = v_new;
    q_ = q_new;
}

// ── predictIMU ────────────────────────────────────────────────────────────────

void ESKF::predictIMU(const Eigen::Vector3d& accel,
                      const Eigen::Vector3d& gyro,
                      double dt,
                      uint64_t stamp_us) {
    if (!initialised_ || dt <= 0.0 || dt > 1.0) return;

    history_[hist_head_] = { saveSnapshot(), accel, gyro, dt, stamp_us };
    hist_head_ = (hist_head_ + 1) % kHistSize;
    if (hist_count_ < kHistSize) ++hist_count_;

    predictIMUCore(accel, gyro, dt);
}

// ── applyMeasPos ──────────────────────────────────────────────────────────────

void ESKF::applyMeasPos(const Eigen::Vector3d& p_meas, double sigma_m) {
    if (!initialised_) return;

    Eigen::Vector3d z = p_meas - p_;

    Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity() * (sigma_m * sigma_m);
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_meas;
    Eigen::Matrix<double,15,3> K = P_ * H.transpose() * S.inverse();
    Eigen::Matrix<double,15,1> dx = K * z;

    injectAndReset(dx);

    Eigen::Matrix<double,15,15> IKH = Eigen::Matrix<double,15,15>::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R_meas * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
}

// ── applyMeasQuat ─────────────────────────────────────────────────────────────

void ESKF::applyMeasQuat(const Eigen::Quaterniond& q_meas, double sigma_rad) {
    if (!initialised_) return;

    Eigen::Quaterniond dq = (q_.conjugate() * q_meas.normalized()).normalized();
    if (dq.w() < 0.0) dq.coeffs() = -dq.coeffs();
    Eigen::Vector3d z = 2.0 * dq.vec();

    Eigen::Matrix<double,3,15> H = Eigen::Matrix<double,3,15>::Zero();
    H.block<3,3>(0,6) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_meas = Eigen::Matrix3d::Identity() * (sigma_rad * sigma_rad);
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_meas;
    Eigen::Matrix<double,15,3> K = P_ * H.transpose() * S.inverse();
    Eigen::Matrix<double,15,1> dx = K * z;

    injectAndReset(dx);

    Eigen::Matrix<double,15,15> IKH = Eigen::Matrix<double,15,15>::Identity() - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R_meas * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
}

// ── snapshot helpers ──────────────────────────────────────────────────────────

EskfSnapshot ESKF::saveSnapshot() const {
    return { p_, v_, ba_, bg_, q_, P_ };
}

void ESKF::restoreSnapshot(const EskfSnapshot& s) {
    p_  = s.p;
    v_  = s.v;
    ba_ = s.ba;
    bg_ = s.bg;
    q_  = s.q;
    P_  = s.P;
}

// ── findSnapshotBefore ────────────────────────────────────────────────────────

int ESKF::findSnapshotBefore(uint64_t stamp_us) const {
    if (hist_count_ == 0) return -1;
    int oldest_phys = (hist_head_ - hist_count_ + kHistSize) % kHistSize;
    for (int k = hist_count_ - 1; k >= 0; --k) {
        int phys = (oldest_phys + k) % kHistSize;
        if (history_[phys].stamp_us <= stamp_us) return phys;
    }
    return -1;
}

// ── replayForward ─────────────────────────────────────────────────────────────
// Walk ring buffer forward from start_idx inclusive.
// The entry at start_idx is replayed first because we restored its pre_state.

void ESKF::replayForward(int start_idx, uint64_t until_us) {
    int oldest_phys   = (hist_head_ - hist_count_ + kHistSize) % kHistSize;
    int start_logical = (start_idx - oldest_phys + kHistSize) % kHistSize;
    uint64_t prev_stamp = 0;
    for (int k = start_logical; k < hist_count_; ++k) {
        int phys = (oldest_phys + k) % kHistSize;
        const ImuHistoryEntry& e = history_[phys];
        if (e.stamp_us > until_us) break;
        if (prev_stamp > 0 && e.stamp_us > prev_stamp + 50000) break;  // 50 ms gap
        predictIMUCore(e.acc_raw, e.gyr_raw, e.dt_s);
        prev_stamp = e.stamp_us;
    }
}

// ── measureCamera ─────────────────────────────────────────────────────────────

void ESKF::measureCamera(const Eigen::Vector3d& p_meas, double sigma_pos,
                          const Eigen::Quaterniond& q_meas, double sigma_att,
                          uint64_t meas_stamp_us, uint64_t now_us) {
    if (!initialised_) return;
    int idx = -1;
    if (meas_stamp_us > 0 && now_us > meas_stamp_us)
        idx = findSnapshotBefore(meas_stamp_us);
    if (idx >= 0) {
        restoreSnapshot(history_[idx].pre_state);
        applyMeasPos(p_meas, sigma_pos);
        applyMeasQuat(q_meas, sigma_att);
        replayForward(idx, now_us);
    } else {
        applyMeasPos(p_meas, sigma_pos);
        applyMeasQuat(q_meas, sigma_att);
    }
}

// ── injectAndReset ────────────────────────────────────────────────────────────

void ESKF::injectAndReset(const Eigen::Matrix<double,15,1>& dx) {
    p_  += dx.segment<3>(0);
    v_  += dx.segment<3>(3);

    Eigen::Vector3d dtheta = dx.segment<3>(6);
    double dn = dtheta.norm();
    Eigen::Quaterniond dq;
    if (dn > 1e-8) {
        dq = Eigen::Quaterniond{std::cos(dn * 0.5),
                                 (dtheta / dn * std::sin(dn * 0.5))(0),
                                 (dtheta / dn * std::sin(dn * 0.5))(1),
                                 (dtheta / dn * std::sin(dn * 0.5))(2)};
    } else {
        dq = Eigen::Quaterniond{1.0, 0.5*dtheta(0), 0.5*dtheta(1), 0.5*dtheta(2)};
    }
    q_ = (q_ * dq).normalized();

    ba_ += dx.segment<3>(9);
    bg_ += dx.segment<3>(12);
}

// ── getState ──────────────────────────────────────────────────────────────────

EskfState ESKF::getState() const {
    EskfState s;
    s.position = p_;
    s.velocity = v_;
    s.attitude = q_;
    return s;
}

// ── updateNoise ───────────────────────────────────────────────────────────────

void ESKF::updateNoise(const EskfNoise& n) {
    noise_ = n;
}
