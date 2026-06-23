// ui/BackendModel.h
// QObject that exposes all backend state to QML via Q_PROPERTYs.
#pragma once
#include <QObject>
#include <QJsonObject>
#include <QJsonArray>
#include <QString>
#include <QVariantMap>

#include "SharedMemoryReader.h"
#include "SocketClient.h"

class FrameProvider;

class BackendModel : public QObject
{
    Q_OBJECT

    Q_PROPERTY(double posX   READ posX   NOTIFY poseUpdated)
    Q_PROPERTY(double posY   READ posY   NOTIFY poseUpdated)
    Q_PROPERTY(double posZ   READ posZ   NOTIFY poseUpdated)
    Q_PROPERTY(double roll   READ roll   NOTIFY poseUpdated)
    Q_PROPERTY(double pitch  READ pitch  NOTIFY poseUpdated)
    Q_PROPERTY(double yaw    READ yaw    NOTIFY poseUpdated)
    Q_PROPERTY(double kalmanVariance READ kalmanVariance NOTIFY poseUpdated)
    Q_PROPERTY(float eskfVelNorm READ eskfVelNorm NOTIFY poseUpdated)
    Q_PROPERTY(float eskfBaNorm  READ eskfBaNorm  NOTIFY poseUpdated)
    Q_PROPERTY(float eskfBgNorm  READ eskfBgNorm  NOTIFY poseUpdated)

    Q_PROPERTY(bool  calibrationComplete READ calibrationComplete NOTIFY statusUpdated)
    Q_PROPERTY(bool  trackingActive      READ trackingActive      NOTIFY statusUpdated)
    Q_PROPERTY(bool  imuOnlyActive       READ imuOnlyActive       NOTIFY statusUpdated)
    Q_PROPERTY(bool  trackingLost        READ trackingLost        NOTIFY statusUpdated)
    Q_PROPERTY(bool  imuOk               READ imuOk               NOTIFY statusUpdated)
    Q_PROPERTY(bool  freedConnected      READ freedConnected      NOTIFY statusUpdated)
    Q_PROPERTY(float freedHz             READ freedHz             NOTIFY statusUpdated)
    Q_PROPERTY(float freedLatency        READ freedLatency        NOTIFY statusUpdated)
    Q_PROPERTY(float visionHz            READ visionHz            NOTIFY statusUpdated)
    Q_PROPERTY(float poseHz              READ poseHz              NOTIFY statusUpdated)
    Q_PROPERTY(QString statusMsg         READ statusMsg           NOTIFY statusUpdated)

    Q_PROPERTY(QString freedIp      READ freedIp      NOTIFY settingsUpdated)
    Q_PROPERTY(int     freedPort    READ freedPort    NOTIFY settingsUpdated)
    Q_PROPERTY(bool    freedEnabled READ freedEnabled NOTIFY settingsUpdated)
    Q_PROPERTY(double  cameraOffsetX READ cameraOffsetX NOTIFY settingsUpdated)
    Q_PROPERTY(double  cameraOffsetY READ cameraOffsetY NOTIFY settingsUpdated)
    Q_PROPERTY(double  cameraOffsetZ READ cameraOffsetZ NOTIFY settingsUpdated)
    Q_PROPERTY(QString detectorDebugMode READ detectorDebugMode NOTIFY settingsUpdated)
    Q_PROPERTY(QVariantMap configValues READ configValues NOTIFY configUpdated)

    Q_PROPERTY(bool socketConnected READ socketConnected NOTIFY socketConnectedChanged)
    Q_PROPERTY(int frameCounter READ frameCounter NOTIFY frameCounterChanged)

    Q_PROPERTY(QJsonArray scaleFiles READ scaleFiles NOTIFY scaleFilesChanged)
    Q_PROPERTY(QJsonArray mapFiles   READ mapFiles   NOTIFY mapFilesChanged)
    Q_PROPERTY(QString currentMapName READ currentMapName NOTIFY starMapChanged)
    Q_PROPERTY(QJsonArray currentMapPoints READ currentMapPoints NOTIFY starMapChanged)

    Q_PROPERTY(int     progressPct  READ progressPct  NOTIFY progressChanged)
    Q_PROPERTY(QString progressStep READ progressStep NOTIFY progressChanged)

    Q_PROPERTY(QJsonArray scanDetectedPoints READ scanDetectedPoints NOTIFY scanOverlayUpdated)
    Q_PROPERTY(QJsonArray scanConfirmedPoints READ scanConfirmedPoints NOTIFY scanOverlayUpdated)
    Q_PROPERTY(int scanFrame READ scanFrame NOTIFY scanOverlayUpdated)
    Q_PROPERTY(int scanDetectedCount READ scanDetectedCount NOTIFY scanOverlayUpdated)
    Q_PROPERTY(int scanConfirmedCount READ scanConfirmedCount NOTIFY scanOverlayUpdated)
    Q_PROPERTY(int scanTotalCount READ scanTotalCount NOTIFY scanOverlayUpdated)
    Q_PROPERTY(int scanMatchedCount READ scanMatchedCount NOTIFY scanOverlayUpdated)
    Q_PROPERTY(int scanNewCount READ scanNewCount NOTIFY scanOverlayUpdated)

public:
    explicit BackendModel(QObject* parent = nullptr);
    ~BackendModel() override;

    void setFrameProvider(FrameProvider* fp);

    double posX()  const { return m_x; }
    double posY()  const { return m_y; }
    double posZ()  const { return m_z; }
    double roll()  const { return m_roll; }
    double pitch() const { return m_pitch; }
    double yaw()   const { return m_yaw; }
    double kalmanVariance() const { return m_kalmanVariance; }
    float eskfVelNorm() const { return m_eskfVelNorm; }
    float eskfBaNorm()  const { return m_eskfBaNorm; }
    float eskfBgNorm()  const { return m_eskfBgNorm; }

    bool    calibrationComplete() const { return m_calibrationComplete; }
    bool    trackingActive()      const { return m_trackingActive; }
    bool    imuOnlyActive()       const { return m_imuOnlyActive; }
    bool    trackingLost()        const { return m_trackingLost; }
    bool    imuOk()               const { return m_imuOk; }
    bool    freedConnected()      const { return m_freedConnected; }
    float   freedHz()             const { return m_freedHz; }
    float   freedLatency()        const { return m_freedLatencyMs; }
    float   visionHz()            const { return m_visionHz; }
    float   poseHz()              const { return m_poseHz; }
    QString statusMsg()           const { return m_statusMsg; }

    QString freedIp()      const { return m_freedIp; }
    int     freedPort()    const { return m_freedPort; }
    bool    freedEnabled() const { return m_freedEnabled; }
    double  cameraOffsetX() const { return m_cameraOffsetX; }
    double  cameraOffsetY() const { return m_cameraOffsetY; }
    double  cameraOffsetZ() const { return m_cameraOffsetZ; }
    QString detectorDebugMode() const { return m_detectorDebugMode; }
    QVariantMap configValues() const { return m_configValues; }

    bool       socketConnected() const { return m_socketConnected; }
    int        frameCounter()    const;
    QJsonArray scaleFiles()      const { return m_scaleFiles; }
    QJsonArray mapFiles()        const { return m_mapFiles; }
    QString    currentMapName()  const { return m_currentMapName; }
    QJsonArray currentMapPoints() const { return m_currentMapPoints; }
    int        progressPct()     const { return m_progressPct; }
    QString    progressStep()    const { return m_progressStep; }

    QJsonArray scanDetectedPoints() const { return m_scanDetectedPoints; }
    QJsonArray scanConfirmedPoints() const { return m_scanConfirmedPoints; }
    int scanFrame() const { return m_scanFrame; }
    int scanDetectedCount() const { return m_scanDetectedCount; }
    int scanConfirmedCount() const { return m_scanConfirmedCount; }
    int scanTotalCount() const { return m_scanTotalCount; }
    int scanMatchedCount() const { return m_scanMatchedCount; }
    int scanNewCount() const { return m_scanNewCount; }

    Q_INVOKABLE void requestStatus()                          { m_socket->requestStatus(); }
    Q_INVOKABLE void requestConfig()                          { m_socket->requestConfig(); }
    Q_INVOKABLE void requestFileList(const QString& type)     { m_socket->requestFileList(type); }
    Q_INVOKABLE void requestCurrentMap()                      { m_socket->requestCurrentMap(); }
    Q_INVOKABLE void startTrack()                             { m_socket->startTrack(); }
    Q_INVOKABLE void stopTracking()                           { m_socket->stop(); }
    Q_INVOKABLE void startImuOnly()                           { m_socket->startImuOnly(); }
    Q_INVOKABLE void stopImuOnly()                            { m_socket->stopImuOnly(); }
    Q_INVOKABLE void startTrackVisionOnly()                   { m_socket->startTrackVisionOnly(); }
    Q_INVOKABLE void anchorPreview()                          { m_socket->anchorPreview(); }
    Q_INVOKABLE void scaleAnchor(int idA, int idB, double distM) { m_socket->scaleAnchor(idA, idB, distM); }
    Q_INVOKABLE void calibrateScale(const QString& file = "") { m_socket->calibrateScale(file); }
    Q_INVOKABLE void buildMap(const QString& file = "")       { m_socket->buildMap(file); }
    Q_INVOKABLE void stopBuildMap()                           { m_socket->stopBuildMap(); }
    Q_INVOKABLE void deleteMap(const QString& file)           { m_socket->deleteMap(file); }
    Q_INVOKABLE void setDetectorDebugMode(const QString& mode) { m_socket->setDetectorDebugMode(mode); }
    Q_INVOKABLE void setGridVisible(bool v)                    { m_socket->setGridVisible(v); }
    Q_INVOKABLE void setSettings(const QString& ip, int port) { m_socket->setSettings(ip, port); }
    Q_INVOKABLE void setCameraOffset(double x, double y, double z) { m_socket->setCameraOffset(x, y, z); }
    Q_INVOKABLE void setRuntimeFlags(bool freedEnabled, bool imuEnabled) { m_socket->setRuntimeFlags(freedEnabled, imuEnabled); }
    Q_INVOKABLE void setConfig(const QVariantMap& values)      { m_socket->setConfig(values); }
    Q_INVOKABLE void calibConfirm()                           { m_socket->calibConfirm(); }
    Q_INVOKABLE void stopTrackingIfActive();
    Q_INVOKABLE void shutdownSystem();
    Q_INVOKABLE void rebootSystem();
    Q_INVOKABLE void clearScanOverlay();

signals:
    void poseUpdated();
    void statusUpdated();
    void settingsUpdated();
    void configUpdated();
    void socketConnectedChanged();
    void frameCounterChanged();
    void scaleFilesChanged();
    void mapFilesChanged();
    void starMapChanged();
    void progressChanged();
    void scanOverlayUpdated();
    void errorOccurred(const QString& msg);
    void scaleDone(const QString& file, double height_m);
    void calibrationDone(const QString& file, int stars);
    void calibPromptReceived(int step, const QString& msg);
    void anchorPreviewDone(bool ok, int matched);
    void scaleAnchorDone(double scaleFactor);

private slots:
    void onPoseUpdated(const PoseSnapshot& pose);
    void onStatusUpdated(const StatusSnapshot& status);
    void onFrameUpdated(const QImage& frame);
    void onSocketStatusReceived(const QJsonObject& obj);
    void onConfigReceived(const QVariantMap& values);
    void onFileListReceived(const QString& type, const QJsonArray& files);
    void onStarMapReceived(const QString& name, const QJsonArray& points);
    void onProgressReceived(int pct, const QString& step);
    void onSocketConnectedChanged(bool connected);
    void onMapScanStatusReceived(int frame,
                                 int detected,
                                 int confirmed,
                                 int total,
                                 int matched,
                                 int added,
                                 const QJsonArray& detections,
                                 const QJsonArray& confirmedPoints);

private:
    SharedMemoryReader* m_shm    = nullptr;
    SocketClient*       m_socket = nullptr;
    FrameProvider*      m_frame  = nullptr;

    double m_x = 0, m_y = 0, m_z = 0;
    double m_roll = 0, m_pitch = 0, m_yaw = 0;
    double m_kalmanVariance = 0;
    float m_eskfVelNorm = 0.f;
    float m_eskfBaNorm  = 0.f;
    float m_eskfBgNorm  = 0.f;

    bool    m_calibrationComplete = false;
    bool    m_trackingActive      = false;
    bool    m_imuOnlyActive       = false;
    bool    m_trackingLost        = false;
    bool    m_imuOk               = false;
    bool    m_freedConnected      = false;
    float   m_freedHz             = 0.f;
    float   m_freedLatencyMs      = 0.f;
    float   m_visionHz            = 0.f;
    float   m_poseHz              = 0.f;
    QString m_statusMsg;

    QString m_freedIp   = "192.168.1.100";
    int     m_freedPort = 40000;
    bool    m_freedEnabled = true;
    double  m_cameraOffsetX = 0.0;
    double  m_cameraOffsetY = 0.0;
    double  m_cameraOffsetZ = 0.0;
    QString m_detectorDebugMode = "off";
    QVariantMap m_configValues;

    bool       m_socketConnected = false;
    QJsonArray m_scaleFiles;
    QJsonArray m_mapFiles;
    QString    m_currentMapName;
    QJsonArray m_currentMapPoints;
    int        m_progressPct  = -1;
    QString    m_progressStep;

    QJsonArray m_scanDetectedPoints;
    QJsonArray m_scanConfirmedPoints;
    int m_scanFrame = 0;
    int m_scanDetectedCount = 0;
    int m_scanConfirmedCount = 0;
    int m_scanTotalCount = 0;
    int m_scanMatchedCount = 0;
    int m_scanNewCount = 0;
};
