// ui/main.cpp
// Entry point for startracker-ui.
// CPU affinity pinning to Core 0 is optional — see ST_UI_PIN_CORE below.
// Registers FrameProvider as image provider.
// Creates BackendModel and exposes it to QML as "backend".
// Starts the QML engine fullscreen.
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickWindow>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <QDir>
#include <QRegularExpression>
#include <QHostAddress>
#include <QImage>

#include <sched.h>
#include <unistd.h>

#include "FrameProvider.h"
#include "BackendModel.h"
#include "SharedMemoryReader.h"

static void maybePinToCore0()
{
    // CPU affinity pinning to Core 0 is optional.
    // Only enable if profiling shows the UI competing with backend cores.
    // By default, leave affinity unset (OS scheduler handles it).
#ifdef ST_UI_PIN_CORE
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) != 0) {
        // Non-fatal: log and continue. On non-Pi platforms this may fail.
        perror("sched_setaffinity(core 0)");
    }
#endif
}

int main(int argc, char* argv[])
{
    maybePinToCore0();

    // Wayland kiosk: no window decorations
    qputenv("QT_QPA_PLATFORM", "wayland");
    qputenv("QT_WAYLAND_DISABLE_WINDOWDECORATION", "1");
    // Use Basic style (no Material dependency required on Pi)
    qputenv("QT_QUICK_CONTROLS_STYLE", "Basic");

    // Register custom types for cross-thread signal/slot passing
    qRegisterMetaType<PoseSnapshot>("PoseSnapshot");
    qRegisterMetaType<StatusSnapshot>("StatusSnapshot");

    QGuiApplication app(argc, argv);
    app.setApplicationName("startracker-ui");

    // FrameProvider lifetime is managed by the QML engine after addImageProvider()
    auto* frameProvider = new FrameProvider();

    BackendModel backendModel;
    backendModel.setFrameProvider(frameProvider);

    QQmlApplicationEngine engine;

    // Expose BackendModel to QML as context property "backend"
    engine.rootContext()->setContextProperty("backend", &backendModel);

    // Register FrameProvider so QML can use source: "image://frame/..."
    // The engine takes ownership of the provider pointer.
    engine.addImageProvider("frame", frameProvider);

    // Qt6 qt_add_qml_module embeds QML files under the URI prefix (:/StarTrackerUI/).
    // The /qt/qml/ sub-path is only used for the QML import registry, not for
    // direct engine.load() calls.
    engine.load(QUrl(QStringLiteral("qrc:/StarTrackerUI/qml/main.qml")));

    if (engine.rootObjects().isEmpty())
        return -1;

    QQuickWindow* window = qobject_cast<QQuickWindow*>(engine.rootObjects().first());

    if (window) {
        // Screenshot server: listens on localhost:40001 for "screenshot <label>\n"
        // Bound to loopback only — not exposed externally.
        auto* screenshotSrv = new QTcpServer(&app);
        if (screenshotSrv->listen(QHostAddress::LocalHost, 40001)) {
            QObject::connect(screenshotSrv, &QTcpServer::newConnection,
                             [screenshotSrv, window]() {
                QTcpSocket* sock = screenshotSrv->nextPendingConnection();
                QObject::connect(sock, &QTcpSocket::readyRead,
                                 [sock, window]() {
                    while (sock->canReadLine()) {
                        const QString line = sock->readLine().trimmed();
                        const QStringList parts = line.split(QLatin1Char(' '));
                        if (parts.isEmpty() || parts[0] != QLatin1String("screenshot")) {
                            sock->write("err: expected 'screenshot <label>'\n");
                            sock->flush();
                            continue;
                        }
                        QString label = (parts.size() >= 2) ? parts[1] : QStringLiteral("screenshot");
                        label.replace(QRegularExpression(QStringLiteral("[^a-zA-Z0-9_-]")),
                                      QStringLiteral("_"));

                        const QString outDir  = QDir::homePath() + QStringLiteral("/.startracker/captures/");
                        const QString outPath = outDir + label + QStringLiteral(".png");
                        QDir().mkpath(outDir);

                        // Defer grab by one event-loop tick so QML finishes
                        // rendering the current screen before we read the framebuffer.
                        QTimer::singleShot(0, sock, [window, outPath, sock]() {
                            const QImage img  = window->grabWindow();
                            const bool  saved = img.save(outPath);
                            sock->write(saved
                                ? ("ok: " + outPath + "\n").toUtf8()
                                : QByteArrayLiteral("err: save mislukt\n"));
                            sock->flush();
                            sock->disconnectFromHost();
                        });
                    }
                });
                QObject::connect(sock, &QTcpSocket::disconnected, sock, &QObject::deleteLater);
            });
        }
    }

    // Do NOT call showFullScreen() here.
    // Under Weston kiosk-shell, the compositor already sends
    // xdg_toplevel.configure(fullscreen) as soon as the surface is created.
    // Calling showFullScreen() again triggers a second configure round-trip that
    // causes Qt's frame-callback timeout ("Didn't receive frame callback in time"),
    // which marks the window inexposed and stops the render loop — black screen.

    return app.exec();
}
