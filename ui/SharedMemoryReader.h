// ui/SharedMemoryReader.h
// QThread that polls /dev/shm/startracker at 20 Hz.
// Reads PoseData and FrameData using seqlock protocol.
// All emitted signals are connected via Qt::QueuedConnection
// (emitted from this thread, consumed by the main thread).
#pragma once
#include <QThread>
#include <QImage>
#include <QTimer>

// Lightweight POD snapshots — no Qt types, no shared_ptr.
// Copied out of SharedBlock under the seqlock and passed across the signal boundary.
struct PoseSnapshot {
    double x = 0, y = 0, z = 0;
    double roll = 0, pitch = 0, yaw = 0;
    double kalman_variance = 0;
    uint64_t timestamp_us = 0;
    float eskf_vel_norm = 0.f;
    float eskf_ba_norm  = 0.f;
    float eskf_bg_norm  = 0.f;
};

struct StatusSnapshot {
    bool calibration_complete = false;
    bool tracking_active      = false;
    bool tracking_lost        = false;
    bool imu_ok               = false;
    bool freed_connected      = false;
    float freed_hz            = 0.f;
    float freed_latency_ms    = 0.f;
    float vision_hz           = 0.f;
    float pose_hz             = 0.f;
};

class SharedMemoryReader : public QThread
{
    Q_OBJECT

public:
    explicit SharedMemoryReader(QObject* parent = nullptr);
    ~SharedMemoryReader() override;

    // Call before start() if you want a custom shm path (e.g. for testing)
    void setShmPath(const QString& path);

    void stop();

protected:
    void run() override;

signals:
    void poseUpdated(const PoseSnapshot& pose);
    void frameUpdated(const QImage& frame);
    void statusUpdated(const StatusSnapshot& status);

private:
    void tryOpenShm();
    void poll();

    QString   m_shmPath    = "/dev/shm/startracker";
    void*     m_mapAddr    = nullptr;
    int       m_shmFd      = -1;

    uint64_t       m_lastPoseSeq  = 0;
    uint64_t       m_lastFrameSeq = 0;
    StatusSnapshot m_prevStatus_{};  // zero-initialised; first cycle always emits
};
