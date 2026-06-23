// ui/BackendModel.cpp
#include "BackendModel.h"
#include "FrameProvider.h"
#include <QDebug>

BackendModel::BackendModel(QObject* parent)
    : QObject(parent)
    , m_shm(new SharedMemoryReader(this))
    , m_socket(new SocketClient(this))
{
    connect(m_shm, &SharedMemoryReader::poseUpdated,   this, &BackendModel::onPoseUpdated,   Qt::QueuedConnection);
    connect(m_shm, &SharedMemoryReader::statusUpdated, this, &BackendModel::onStatusUpdated, Qt::QueuedConnection);
    connect(m_shm, &SharedMemoryReader::frameUpdated,  this, &BackendModel::onFrameUpdated,  Qt::QueuedConnection);

    connect(m_socket, &SocketClient::statusReceived,   this, &BackendModel::onSocketStatusReceived, Qt::QueuedConnection);
    connect(m_socket, &SocketClient::configReceived,   this, &BackendModel::onConfigReceived,       Qt::QueuedConnection);
    connect(m_socket, &SocketClient::fileListReceived, this, &BackendModel::onFileListReceived,     Qt::QueuedConnection);
    connect(m_socket, &SocketClient::starMapReceived,  this, &BackendModel::onStarMapReceived,      Qt::QueuedConnection);
    connect(m_socket, &SocketClient::progressReceived, this, &BackendModel::onProgressReceived,     Qt::QueuedConnection);
    connect(m_socket, &SocketClient::connectedChanged, this, &BackendModel::onSocketConnectedChanged, Qt::QueuedConnection);
    connect(m_socket, &SocketClient::errorReceived,    this, &BackendModel::errorOccurred,           Qt::QueuedConnection);
    connect(m_socket, &SocketClient::scaleDone,        this, &BackendModel::scaleDone,               Qt::QueuedConnection);
    connect(m_socket, &SocketClient::calibrationDone,  this, &BackendModel::calibrationDone,         Qt::QueuedConnection);
    connect(m_socket, &SocketClient::calibPromptReceived, this, &BackendModel::calibPromptReceived,  Qt::QueuedConnection);
    connect(m_socket, &SocketClient::mapScanStatusReceived, this, &BackendModel::onMapScanStatusReceived, Qt::QueuedConnection);
    connect(m_socket, &SocketClient::anchorPreviewDone,    this, &BackendModel::anchorPreviewDone,       Qt::QueuedConnection);
    connect(m_socket, &SocketClient::scaleAnchorDone,      this, &BackendModel::scaleAnchorDone,         Qt::QueuedConnection);

    m_shm->start();
    m_socket->start();
}

BackendModel::~BackendModel()
{
    stopTrackingIfActive();
    m_shm->stop();
    m_socket->quit();
    m_shm->wait();
    m_socket->wait();
}

void BackendModel::setFrameProvider(FrameProvider* fp)
{
    m_frame = fp;
    connect(m_shm, &SharedMemoryReader::frameUpdated, m_frame, &FrameProvider::updateFrame, Qt::QueuedConnection);
    connect(m_frame, &FrameProvider::frameCounterChanged, this, &BackendModel::frameCounterChanged);
}

int BackendModel::frameCounter() const
{
    return m_frame ? m_frame->frameCounter() : 0;
}

void BackendModel::stopTrackingIfActive()
{
    if (m_trackingActive) {
        m_socket->stop();
    }
}

void BackendModel::shutdownSystem()
{
    stopTrackingIfActive();
    m_socket->shutdownSystem();
}

void BackendModel::rebootSystem()
{
    stopTrackingIfActive();
    m_socket->rebootSystem();
}

void BackendModel::clearScanOverlay()
{
    m_scanDetectedPoints = QJsonArray{};
    m_scanConfirmedPoints = QJsonArray{};
    m_scanFrame = 0;
    m_scanDetectedCount = 0;
    m_scanConfirmedCount = 0;
    m_scanTotalCount = 0;
    m_scanMatchedCount = 0;
    m_scanNewCount = 0;
    emit scanOverlayUpdated();
}

void BackendModel::onPoseUpdated(const PoseSnapshot& pose)
{
    m_x              = pose.x;
    m_y              = pose.y;
    m_z              = pose.z;
    m_roll           = pose.roll;
    m_pitch          = pose.pitch;
    m_yaw            = pose.yaw;
    m_kalmanVariance = pose.kalman_variance;
    m_eskfVelNorm    = pose.eskf_vel_norm;
    m_eskfBaNorm     = pose.eskf_ba_norm;
    m_eskfBgNorm     = pose.eskf_bg_norm;
    emit poseUpdated();
}

void BackendModel::onStatusUpdated(const StatusSnapshot& status)
{
    m_calibrationComplete = status.calibration_complete;
    m_trackingActive      = status.tracking_active;
    m_trackingLost        = status.tracking_lost;
    m_imuOk               = status.imu_ok;
    m_freedConnected      = status.freed_connected;
    m_freedHz             = status.freed_hz;
    m_freedLatencyMs      = status.freed_latency_ms;
    m_visionHz            = status.vision_hz;
    m_poseHz              = status.pose_hz;
    emit statusUpdated();
}

void BackendModel::onFrameUpdated(const QImage& /*frame*/)
{
}

void BackendModel::onSocketStatusReceived(const QJsonObject& obj)
{
    bool changed = false;
    if (obj.contains("freed_ip")) {
        m_freedIp = obj.value("freed_ip").toString();
        changed = true;
    }
    if (obj.contains("freed_port")) {
        m_freedPort = obj.value("freed_port").toInt(40000);
        changed = true;
    }
    if (obj.contains("freed_enabled")) {
        m_freedEnabled = obj.value("freed_enabled").toBool(true);
        changed = true;
    }
    if (obj.contains("camera_offset_x_m")) {
        m_cameraOffsetX = obj.value("camera_offset_x_m").toDouble(0.0);
        changed = true;
    }
    if (obj.contains("camera_offset_y_m")) {
        m_cameraOffsetY = obj.value("camera_offset_y_m").toDouble(0.0);
        changed = true;
    }
    if (obj.contains("camera_offset_z_m")) {
        m_cameraOffsetZ = obj.value("camera_offset_z_m").toDouble(0.0);
        changed = true;
    }
    if (obj.contains("detector_debug_mode")) {
        m_detectorDebugMode = obj.value("detector_debug_mode").toString("off");
        changed = true;
    }
    if (obj.contains("imu_only_active")) {
        m_imuOnlyActive = obj.value("imu_only_active").toBool(false);
        emit statusUpdated();
    }
    if (obj.contains("status")) {
        m_statusMsg = obj.value("status").toString();
        emit statusUpdated();
    }
    if (changed) emit settingsUpdated();
}

void BackendModel::onConfigReceived(const QVariantMap& values)
{
    m_configValues = values;
    emit configUpdated();
}

void BackendModel::onFileListReceived(const QString& type, const QJsonArray& files)
{
    if (type == "scale_calibration") {
        m_scaleFiles = files;
        emit scaleFilesChanged();
    } else if (type == "star_maps") {
        m_mapFiles = files;
        emit mapFilesChanged();
    }
}

void BackendModel::onStarMapReceived(const QString& name, const QJsonArray& points)
{
    m_currentMapName = name;
    m_currentMapPoints = points;
    emit starMapChanged();
}

void BackendModel::onProgressReceived(int pct, const QString& step)
{
    m_progressPct = pct;
    m_progressStep = step;
    emit progressChanged();
}

void BackendModel::onSocketConnectedChanged(bool connected)
{
    m_socketConnected = connected;
    emit socketConnectedChanged();
    if (connected) {
        m_socket->requestStatus();
        m_socket->requestConfig();
        m_socket->requestCurrentMap();
    }
}

void BackendModel::onMapScanStatusReceived(int frame,
                                           int detected,
                                           int confirmed,
                                           int total,
                                           int matched,
                                           int added,
                                           const QJsonArray& detections,
                                           const QJsonArray& confirmedPoints)
{
    m_scanFrame = frame;
    m_scanDetectedCount = detected;
    m_scanConfirmedCount = confirmed;
    m_scanTotalCount = total;
    m_scanMatchedCount = matched;
    m_scanNewCount = added;
    m_scanDetectedPoints = detections;
    m_scanConfirmedPoints = confirmedPoints;
    emit scanOverlayUpdated();
}
