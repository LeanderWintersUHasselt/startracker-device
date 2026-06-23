// ui/SharedMemoryReader.cpp
// Adapted for actual shared/ipc.h field names:
//   - pose_confidence  (not kalman_variance)
//   - PREVIEW_MAX_BYTES (not PREVIEW_BYTES)
//   - preview_frame_id is uint64_t
//   - no status_msg field in SharedBlock
#include "SharedMemoryReader.h"
#include "ipc.h"

#include <QDebug>

#include <cerrno>
#include <cstring>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

SharedMemoryReader::SharedMemoryReader(QObject* parent)
    : QThread(parent)
{}

SharedMemoryReader::~SharedMemoryReader()
{
    stop();
    wait();
    if (m_mapAddr && m_mapAddr != MAP_FAILED)
        munmap(m_mapAddr, sizeof(SharedBlock));
    if (m_shmFd >= 0)
        close(m_shmFd);
}

void SharedMemoryReader::setShmPath(const QString& path)
{
    m_shmPath = path;
}

void SharedMemoryReader::stop()
{
    quit();
}

void SharedMemoryReader::tryOpenShm()
{
    // Convert "/dev/shm/startracker" → shm_open name "/startracker"
    // shm_open expects a name starting with '/', not a full path.
    QString name = m_shmPath;
    if (name.startsWith("/dev/shm"))
        name = name.mid(8);  // strip "/dev/shm" → "/startracker"

    m_shmFd = shm_open(name.toLocal8Bit().constData(), O_RDONLY, 0);
    if (m_shmFd < 0) {
        // Backend not yet started — will retry next poll cycle
        return;
    }

    m_mapAddr = mmap(nullptr, sizeof(SharedBlock), PROT_READ, MAP_SHARED, m_shmFd, 0);
    if (m_mapAddr == MAP_FAILED) {
        qWarning() << "SharedMemoryReader: mmap failed:" << strerror(errno);
        close(m_shmFd);
        m_shmFd   = -1;
        m_mapAddr = nullptr;
    }
}

void SharedMemoryReader::poll()
{
    if (m_mapAddr == nullptr || m_mapAddr == MAP_FAILED) {
        tryOpenShm();
        return;  // Not yet available — silently skip this tick
    }

    auto* block = static_cast<const SharedBlock*>(m_mapAddr);

    // --- Read pose (seqlock) ---
    PoseSnapshot pose;
    uint64_t seq1, seq2;
    do {
        seq1 = block->pose_seq.load(std::memory_order_acquire);
        if (seq1 & 1) continue;  // writer active, spin
        pose.x              = block->x;
        pose.y              = block->y;
        pose.z              = block->z;
        pose.roll           = block->roll;
        pose.pitch          = block->pitch;
        pose.yaw            = block->yaw;
        pose.kalman_variance = block->pose_confidence;  // actual field name
        pose.timestamp_us   = block->pose_timestamp_us;
        pose.eskf_vel_norm  = block->eskf_vel_norm.load(std::memory_order_relaxed);
        pose.eskf_ba_norm   = block->eskf_ba_norm.load(std::memory_order_relaxed);
        pose.eskf_bg_norm   = block->eskf_bg_norm.load(std::memory_order_relaxed);
        seq2 = block->pose_seq.load(std::memory_order_acquire);
    } while (seq1 != seq2);

    if (seq1 != m_lastPoseSeq) {
        m_lastPoseSeq = seq1;
        emit poseUpdated(pose);
    }

    // --- Read status atomics (relaxed — eventual consistency is fine) ---
    StatusSnapshot status;
    status.calibration_complete = block->calibration_complete.load(std::memory_order_relaxed);
    status.tracking_active      = block->tracking_active.load(std::memory_order_relaxed);
    status.tracking_lost        = block->tracking_lost.load(std::memory_order_relaxed);
    status.imu_ok               = block->imu_ok.load(std::memory_order_relaxed);
    status.freed_connected      = block->freed_connected.load(std::memory_order_relaxed);
    status.freed_hz             = block->freed_hz.load(std::memory_order_relaxed);
    status.freed_latency_ms     = block->freed_latency_ms.load(std::memory_order_relaxed);
    status.vision_hz            = block->vision_hz.load(std::memory_order_relaxed);
    status.pose_hz              = block->pose_hz.load(std::memory_order_relaxed);
    // Note: SharedBlock has no status_msg field — omitted intentionally

    if (std::memcmp(&status, &m_prevStatus_, sizeof(StatusSnapshot)) != 0) {
        m_prevStatus_ = status;
        emit statusUpdated(status);
    }

    // --- Read frame (seqlock) ---
    // Backend writes raw 512×288 grayscale luma bytes — no JPEG decode needed.
    uint64_t fseq1, fseq2;
    uint32_t w = 0, h = 0;
    static thread_local std::vector<uint8_t> rawBuf;

    do {
        fseq1 = block->preview_seq.load(std::memory_order_acquire);
        if (fseq1 & 1) continue;
        w = block->preview_w;
        h = block->preview_h;
        if (w > 0 && h > 0 && w * h * 3 <= PREVIEW_MAX_BYTES) {
            rawBuf.resize(w * h * 3);
            memcpy(rawBuf.data(), block->preview_bytes, w * h * 3);
        }
        fseq2 = block->preview_seq.load(std::memory_order_acquire);
    } while (fseq1 != fseq2);

    if (fseq1 != m_lastFrameSeq && w > 0 && h > 0) {
        m_lastFrameSeq = fseq1;
        // QImage with raw pointer does NOT copy data — must call .copy() to get
        // an owned QImage that is safe to emit across the queued connection boundary.
        emit frameUpdated(
            QImage(rawBuf.data(), static_cast<int>(w), static_cast<int>(h),
                   static_cast<int>(w * 3), QImage::Format_BGR888).copy()
        );

    }
}

void SharedMemoryReader::run()
{
    // 20 Hz poll timer — lives in this thread's event loop
    QTimer timer;
    timer.setInterval(50);  // 20 Hz — sufficient for preview; keeps frontend CPU low
    timer.setTimerType(Qt::PreciseTimer);
    // DirectConnection: timer and this->poll() both live in this worker thread.
    // Without it, AutoConnection would queue poll() to the main thread (QThread
    // object affinity), defeating the purpose of the worker thread entirely.
    connect(&timer, &QTimer::timeout, this, &SharedMemoryReader::poll, Qt::DirectConnection);
    timer.start();

    exec();  // run this thread's event loop until stop() calls quit()
}
