#pragma once
#include <opencv2/opencv.hpp>

// RigidTransform — SE(3) element: rotation + translation.
//
// Convention: p_dst = R * p_src + t
//
// Named instances:
//   T_world_cam  : camera pose in world frame  (R_wc, t_wc)
//   T_cam_world  : OpenCV solvePnP output      (R_cw, t_cw)
//   T_cam_imu    : Kalibr extrinsic            (R_ci, t_ci)
//   T_world_imu  : T_world_cam * T_cam_imu

struct RigidTransform {
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Vec3d   t = {0.0, 0.0, 0.0};

    // Apply: p_dst = R * p_src + t
    cv::Vec3d apply(const cv::Vec3d& p) const { return R * p + t; }
};

// ── Helpers ───────────────────────────────────────────────────────────────────

// T^{-1}: p_src = R^T * (p_dst - t)
inline RigidTransform inverse(const RigidTransform& T) {
    RigidTransform inv;
    inv.R = T.R.t();
    inv.t = -(inv.R * T.t);
    return inv;
}

// T_ac = T_ab * T_bc  (chain: A←B←C)
inline RigidTransform compose(const RigidTransform& T_ab, const RigidTransform& T_bc) {
    RigidTransform T_ac;
    T_ac.R = T_ab.R * T_bc.R;
    T_ac.t = T_ab.R * T_bc.t + T_ab.t;
    return T_ac;
}

// OpenCV solvePnP returns rvec/tvec representing T_cam_world (p_cam = R_cw * p_world + t_cw).
// This converts to T_world_cam (camera pose in world frame).
inline RigidTransform T_world_cam_from_rvec_tvec(const cv::Mat& rvec, const cv::Mat& tvec) {
    cv::Matx33d R_cw;
    cv::Rodrigues(rvec, R_cw);
    cv::Vec3d t_cw(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    RigidTransform T_cam_world;
    T_cam_world.R = R_cw;
    T_cam_world.t = t_cw;

    return inverse(T_cam_world);
}

// Convert T_world_cam back to rvec/tvec (T_cam_world) for use with projectPoints.
inline void T_world_cam_to_rvec_tvec(const RigidTransform& T_wc,
                                      cv::Mat& rvec, cv::Mat& tvec) {
    RigidTransform T_cw = inverse(T_wc);
    cv::Rodrigues(cv::Mat(T_cw.R), rvec);
    tvec = (cv::Mat_<double>(3, 1) << T_cw.t[0], T_cw.t[1], T_cw.t[2]);
}

// Extract Euler angles from T_world_cam for output/FreeD.
// Returns yaw/pitch/roll in degrees (ZYX convention).
// Sign flips for FreeD/UI convention must be applied AFTER this call, not before.
struct EulerAngles { double yaw_deg, pitch_deg, roll_deg; };

inline EulerAngles euler_from_R(const cv::Matx33d& R) {
    // ZYX: yaw around Z, pitch around Y, roll around X
    double yaw   = std::atan2(R(1, 0), R(0, 0));
    double pitch = std::atan2(-R(2, 0), std::sqrt(R(2, 1)*R(2, 1) + R(2, 2)*R(2, 2)));
    double roll  = std::atan2(R(2, 1), R(2, 2));
    constexpr double kRad2Deg = 180.0 / CV_PI;
    return { yaw * kRad2Deg, pitch * kRad2Deg, roll * kRad2Deg };
}
