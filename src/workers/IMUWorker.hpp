#pragma once

#include "SharedState.hpp"
#include "ipc/SharedMemoryServer.hpp"
#include "common/Config.hpp"

class IMUWorker {
public:
    IMUWorker(CameraMeas*         cam_meas,
              VisionPose*         vis_pose,
              ControlState*       ctrl,
              SharedConfig*       config,
              SharedSettings*     settings,
              SharedImuDebug*     imu_debug,
              SharedMemoryServer* blk);
    void run();

    // Bypass path: BNO085 quaternion → FreeD, zero ESKF/filter overhead.
    // Swap run() for runRawImu() in main.cpp to measure true IMU→FreeD latency.
    void runRawImu();

private:
    CameraMeas*         m_cam_meas;
    VisionPose*         m_vis_pose;
    ControlState*       m_ctrl;
    SharedConfig*       m_config;
    SharedSettings*     m_settings;
    SharedImuDebug*     m_imu_debug;
    SharedMemoryServer* m_blk;
};
