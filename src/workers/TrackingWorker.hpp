#pragma once

#include "SharedState.hpp"
#include "ipc/SharedMemoryServer.hpp"
#include "common/Config.hpp"

class TrackingWorker {
public:
    TrackingWorker(FrameSlots*         slots,
                   CameraMeas*         cam_meas,
                   VisionPose*         vis_pose,
                   MapState*           map,
                   ControlState*       ctrl,
                   SharedConfig*       config,
                   SharedMemoryServer* blk);
    void run();

private:
    FrameSlots*         m_slots;
    CameraMeas*         m_cam_meas;
    VisionPose*         m_vis_pose;
    MapState*           m_map;
    ControlState*       m_ctrl;
    SharedConfig*       m_config;
    SharedMemoryServer* m_blk;
};
