// ui/SocketClient.h
// QThread that manages a QLocalSocket connection to /run/startracker/startracker.sock.
// Reconnects automatically with a 1s retry timer on disconnect or failure.
// All command methods are public slots — safe to call from any thread.
// The socket itself lives in this thread's event loop.
#pragma once
#include <QThread>
#include <QLocalSocket>
#include <QTimer>
#include <QJsonObject>
#include <QJsonArray>
#include <QVariantMap>

class SocketClient : public QThread
{
    Q_OBJECT

public:
    explicit SocketClient(QObject* parent = nullptr);
    ~SocketClient() override;

    void setSocketPath(const QString& path);

public slots:
    void sendCommand(const QJsonObject& cmd);

    void requestStatus();
    void requestConfig();
    void requestFileList(const QString& type);
    void requestCurrentMap();
    void startTrack();
    void stop();
    void calibrateScale(const QString& file = "");
    void buildMap(const QString& file = "");
    void stopBuildMap();
    void deleteMap(const QString& file);
    void setDetectorDebugMode(const QString& mode);
    void setGridVisible(bool visible);
    void setSettings(const QString& ip, int port);
    void setCameraOffset(double x, double y, double z);
    void setRuntimeFlags(bool freedEnabled, bool imuEnabled);
    void setConfig(const QVariantMap& values);
    void startImuOnly();
    void stopImuOnly();
    void startTrackVisionOnly();
    void anchorPreview();
    void scaleAnchor(int idA, int idB, double distM);
    void calibConfirm();
    void shutdownSystem();
    void rebootSystem();

signals:
    void statusReceived(const QJsonObject& obj);
    void configReceived(const QVariantMap& values);
    void fileListReceived(const QString& type, const QJsonArray& files);
    void starMapReceived(const QString& name, const QJsonArray& points);
    void progressReceived(int pct, const QString& step);
    void scaleDone(const QString& file, double height_m);
    void calibrationDone(const QString& file, int stars);
    void errorReceived(const QString& msg);
    void connectedChanged(bool connected);
    void calibPromptReceived(int step, const QString& msg);
    void mapScanStatusReceived(int frame,
                               int detected,
                               int confirmed,
                               int total,
                               int matched,
                               int added,
                               const QJsonArray& detections,
                               const QJsonArray& confirmedPoints);
    void anchorPreviewDone(bool ok, int matched);
    void scaleAnchorDone(double scaleFactor);

protected:
    void run() override;

private slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
    void tryConnect();

private:
    void dispatch(const QJsonObject& obj);

    QString        m_socketPath  = "/run/startracker/startracker.sock";
    QLocalSocket*  m_socket      = nullptr;
    QTimer*        m_retryTimer  = nullptr;
    bool           m_connected   = false;
    QByteArray     m_readBuffer;
};
