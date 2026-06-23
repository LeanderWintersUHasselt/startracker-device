#pragma once
#include "geometry/Transform.hpp"
#include <opencv2/opencv.hpp>

// ── CameraIntrinsics ──────────────────────────────────────────────────────────
// Kalibr calibration set: startracker_calib_004

struct CameraIntrinsics {
    cv::Mat   K;           // 3×3 CV_64F camera matrix
    cv::Mat   dist;        // 1×4 CV_64F radtan distortion [k1, k2, p1, p2]
    cv::Size  resolution;  // full sensor resolution

    double fx() const { return K.at<double>(0, 0); }
    double fy() const { return K.at<double>(1, 1); }
    double cx() const { return K.at<double>(0, 2); }
    double cy() const { return K.at<double>(1, 2); }
};

// ── KalibrCalibration ─────────────────────────────────────────────────────────
// All Kalibr-derived values in one place.
// Source: startracker_calib_004 (BNO085 + ArduCam 12MP, 2026-04)

struct KalibrCalibration {
    CameraIntrinsics cam;

    // T_cam_imu: p_cam = R_ci * p_imu + t_ci
    RigidTransform T_cam_imu;

    // t_imu = t_cam + timeshift_cam_imu  [seconds]
    double timeshift_cam_imu = 0.0;

    // Allan6h discrete noise variances for ESKF at 200 Hz (dt = 0.005 s)
    // Conversion: var = (density / sqrt(dt))^2
    struct NoiseParams {
        double var_acc        = 0.0;  // accelerometer white noise variance
        double var_omega      = 0.0;  // gyroscope white noise variance
        double var_acc_bias   = 0.0;  // accelerometer bias variance
        double var_omega_bias = 0.0;  // gyroscope bias variance
        double noise_scale    = 1.0;  // tuning multiplier (2–10 on gyro/accel)
    } noise;
};

// ── Factory: returns hardcoded Kalibr values ──────────────────────────────────
// Use this until a YAML/JSON loader is written.
inline KalibrCalibration make_kalibr_calibration() {
    KalibrCalibration cal;

    // Camera matrix
    cal.cam.K = (cv::Mat_<double>(3, 3) <<
        996.9048237482647,   0.0,             1152.697937938623,
          0.0,             974.058777511983,   644.0649542807813,
          0.0,               0.0,               1.0);

    // Radtan distortion: k1, k2, p1, p2
    cal.cam.dist = (cv::Mat_<double>(1, 4) <<
         0.012078175727637691,
        -0.019427322631540653,
        -0.002925650582159669,
        -0.000839497681398763);

    cal.cam.resolution = cv::Size(2304, 1296);

    // T_cam_imu (Kalibr: p_cam = R_ci * p_imu + t_ci)
    cal.T_cam_imu.R = cv::Matx33d(
        -0.00308372, -0.99999333,  0.00195877,
        -0.99959899,  0.00302736, -0.02815470,
         0.02814859, -0.00204480, -0.99960166);
    cal.T_cam_imu.t = cv::Vec3d(-0.000101286, -0.000704597, -0.014502391);

    // Timeshift: t_imu = t_cam + timeshift_cam_imu
    cal.timeshift_cam_imu = 0.0159050569;  // +15.9 ms

    // Allan6h noise densities → discrete variances at dt = 0.005 s (200 Hz)
    //   sigma_accel = 1.63e-3 m/s^2/sqrt(Hz) → var = (1.63e-3 / sqrt(0.005))^2
    //   sigma_gyro  = 7.2e-5  rad/s/sqrt(Hz) → var = (7.2e-5  / sqrt(0.005))^2
    cal.noise.var_acc        = 5.3138e-4;
    cal.noise.var_omega      = 1.0368e-6;
    cal.noise.var_acc_bias   = 5.5225e-10;  // pow(2.35e-5, 2)
    cal.noise.var_omega_bias = 1.0e-12;     // pow(1.0e-6,  2)
    cal.noise.noise_scale    = 1.0;

    return cal;
}
