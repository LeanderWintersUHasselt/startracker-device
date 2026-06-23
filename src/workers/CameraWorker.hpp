#pragma once

#include "SharedState.hpp"
#include "ipc/SharedMemoryServer.hpp"

class CameraWorker {
public:
    CameraWorker(FrameSlots*         slots,
                 ControlState*       ctrl,
                 SharedConfig*       config,
                 SharedMemoryServer* blk,
                 VisionPose*         vis_pose);
    void run();

private:
    FrameSlots*         m_slots;
    ControlState*       m_ctrl;
    SharedConfig*       m_config;
    SharedMemoryServer* m_blk;
    VisionPose*         m_vis_pose;

    // Gate-filtered grid pose: frozen when stationary, free when moving
    double   m_grid_yaw{0.0};
    double   m_grid_pitch{0.0};
    double   m_grid_roll{0.0};
    double   m_grid_x{0.0};    // camera world position (meters), always current
    double   m_grid_y{0.0};
    uint64_t m_grid_vis_ver{0};
    bool     m_grid_init{false};
    bool     m_grid_moving{false};
    int      m_grid_still_frames{0};
};
