// ui/SocketClient.cpp
#include "SocketClient.h"
#include <QJsonDocument>
#include <QDebug>

SocketClient::SocketClient(QObject* parent)
    : QThread(parent)
{}

SocketClient::~SocketClient()
{
    quit();
    wait();
}

void SocketClient::setSocketPath(const QString& path)
{
    m_socketPath = path;
}

void SocketClient::run()
{
    m_socket     = new QLocalSocket();
    m_retryTimer = new QTimer();
    m_retryTimer->setInterval(1000);
    m_retryTimer->setSingleShot(false);

    connect(m_socket, &QLocalSocket::connected,    this, &SocketClient::onConnected,    Qt::DirectConnection);
    connect(m_socket, &QLocalSocket::disconnected, this, &SocketClient::onDisconnected, Qt::DirectConnection);
    connect(m_socket, &QLocalSocket::readyRead,    this, &SocketClient::onReadyRead,    Qt::DirectConnection);
    connect(m_retryTimer, &QTimer::timeout,        this, &SocketClient::tryConnect,     Qt::DirectConnection);

    tryConnect();
    m_retryTimer->start();

    exec();

    m_retryTimer->stop();
    m_socket->disconnectFromServer();
    delete m_retryTimer;
    delete m_socket;
}

void SocketClient::tryConnect()
{
    if (m_socket->state() == QLocalSocket::UnconnectedState) {
        m_socket->connectToServer(m_socketPath);
    }
}

void SocketClient::onConnected()
{
    qDebug() << "SocketClient: connected to" << m_socketPath;
    m_connected = true;
    emit connectedChanged(true);
    requestStatus();
}

void SocketClient::onDisconnected()
{
    qDebug() << "SocketClient: disconnected — will retry in 1s";
    m_connected = false;
    m_readBuffer.clear();
    emit connectedChanged(false);
}

void SocketClient::onReadyRead()
{
    m_readBuffer += m_socket->readAll();

    while (true) {
        int nl = m_readBuffer.indexOf('\n');
        if (nl < 0) break;

        QByteArray line = m_readBuffer.left(nl).trimmed();
        m_readBuffer.remove(0, nl + 1);

        if (line.size() > 65536) {
            qWarning() << "SocketClient: line too long (" << line.size() << " bytes) — resetting connection";
            m_socket->disconnectFromServer();
            return;
        }

        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(line, &err);
        if (err.error != QJsonParseError::NoError) {
            qWarning() << "SocketClient: JSON parse error:" << err.errorString();
            continue;
        }
        dispatch(doc.object());
    }
}

void SocketClient::dispatch(const QJsonObject& obj)
{
    const QString event = obj.value("event").toString();

    if (event == "status") {
        emit statusReceived(obj);
    } else if (event == "config") {
        emit configReceived(obj.value("values").toObject().toVariantMap());
    } else if (event == "file_list") {
        emit fileListReceived(obj.value("type").toString(),
                              obj.value("files").toArray());
    } else if (event == "star_map") {
        emit starMapReceived(obj.value("name").toString(),
                             obj.value("points").toArray());
    } else if (event == "progress") {
        emit progressReceived(obj.value("pct").toInt(),
                              obj.value("step").toString());
    } else if (event == "scale_done") {
        emit scaleDone(obj.value("file").toString(),
                       obj.value("height_m").toDouble());
    } else if (event == "calibration_done") {
        emit calibrationDone(obj.value("file").toString(),
                             obj.value("stars").toInt());
    } else if (event == "error") {
        emit errorReceived(obj.value("msg").toString());
    } else if (event == "calib_prompt") {
        emit calibPromptReceived(obj.value("step").toInt(),
                                 obj.value("msg").toString());
    } else if (event == "map_scan_status") {
        emit mapScanStatusReceived(obj.value("frame").toInt(),
                                   obj.value("detected").toInt(),
                                   obj.value("confirmed").toInt(),
                                   obj.value("total").toInt(),
                                   obj.value("matched").toInt(),
                                   obj.value("added").toInt(),
                                   obj.value("detections").toArray(),
                                   obj.value("confirmed_points").toArray());
    } else if (event == "anchor_preview_done") {
        emit anchorPreviewDone(obj.value("ok").toBool(false),
                               obj.value("matched").toInt(0));
    } else if (event == "scale_anchor_done") {
        emit scaleAnchorDone(obj.value("scale").toDouble(1.0));
    } else {
        qDebug() << "SocketClient: unknown event:" << event;
    }
}

void SocketClient::sendCommand(const QJsonObject& cmd)
{
    if (!m_connected) return;
    QByteArray data = QJsonDocument(cmd).toJson(QJsonDocument::Compact) + '\n';
    m_socket->write(data);
    m_socket->flush();
}

void SocketClient::requestStatus()
{
    sendCommand({{"cmd", "get_status"}});
}

void SocketClient::requestConfig()
{
    sendCommand({{"cmd", "get_config"}});
}

void SocketClient::requestFileList(const QString& type)
{
    sendCommand({{"cmd", "list_files"}, {"type", type}});
}

void SocketClient::requestCurrentMap()
{
    sendCommand({{"cmd", "get_current_map"}});
}

void SocketClient::startTrack()
{
    sendCommand({{"cmd", "start_track"}});
}

void SocketClient::stop()
{
    sendCommand({{"cmd", "stop"}});
}

void SocketClient::calibrateScale(const QString& file)
{
    QJsonObject cmd{{"cmd", "calibrate_scale"}};
    if (!file.isEmpty()) cmd["file"] = file;
    sendCommand(cmd);
}

void SocketClient::buildMap(const QString& file)
{
    QJsonObject cmd{{"cmd", "build_map"}};
    if (!file.isEmpty()) cmd["file"] = file;
    sendCommand(cmd);
}

void SocketClient::stopBuildMap()
{
    sendCommand({{"cmd", "stop_build_map"}});
}

void SocketClient::deleteMap(const QString& file)
{
    sendCommand({{"cmd", "delete_map"}, {"file", file}});
}

void SocketClient::setDetectorDebugMode(const QString& mode)
{
    sendCommand({{"cmd", "set_detector_debug_mode"}, {"mode", mode}});
}

void SocketClient::setGridVisible(bool visible)
{
    sendCommand({{"cmd", "set_grid_visible"}, {"visible", visible}});
}

void SocketClient::setSettings(const QString& ip, int port)
{
    sendCommand({{"cmd", "set_settings"}, {"freed_ip", ip}, {"freed_port", port}});
}

void SocketClient::setCameraOffset(double x, double y, double z)
{
    sendCommand({{"cmd", "set_settings"},
                 {"camera_offset_x_m", x},
                 {"camera_offset_y_m", y},
                 {"camera_offset_z_m", z}});
}

void SocketClient::setRuntimeFlags(bool freedEnabled, bool imuEnabled)
{
    sendCommand({
        {"cmd", "set_runtime_flags"},
        {"freed_enabled", freedEnabled},
        {"imu_enabled", imuEnabled}
    });
}

void SocketClient::setConfig(const QVariantMap& values)
{
    sendCommand({
        {"cmd", "set_config"},
        {"values", QJsonObject::fromVariantMap(values)}
    });
}

void SocketClient::calibConfirm()
{
    sendCommand({{"cmd", "calib_confirm"}});
}

void SocketClient::shutdownSystem()
{
    sendCommand({{"cmd", "shutdown_system"}});
}

void SocketClient::rebootSystem()
{
    sendCommand({{"cmd", "reboot_system"}});
}

void SocketClient::startImuOnly()
{
    sendCommand({{"cmd", "start_imu_only"}});
}

void SocketClient::stopImuOnly()
{
    sendCommand({{"cmd", "stop_imu_only"}});
}

void SocketClient::startTrackVisionOnly()
{
    sendCommand({{"cmd", "start_track"}, {"imu", false}});
}

void SocketClient::anchorPreview()
{
    sendCommand({{"cmd", "anchor_preview"}});
}

void SocketClient::scaleAnchor(int idA, int idB, double distM)
{
    sendCommand({{"cmd", "scale_anchor"}, {"id_a", idA}, {"id_b", idB}, {"distance_m", distM}});
}
