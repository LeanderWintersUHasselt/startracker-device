// ui/FrameProvider.h
// QQuickImageProvider subclass.
// SharedMemoryReader emits frameUpdated(QImage) → FrameProvider::updateFrame(QImage).
// QML Image binds to source: "image://frame/" + frameCounter.
// Each increment of frameCounter causes Qt to call requestImage() for the new frame.
#pragma once
#include <QQuickImageProvider>
#include <QImage>
#include <QMutex>
#include <atomic>
// Note: do NOT also inherit QObject — in Qt6 QQuickImageProvider already
// inherits QObject (via QQmlImageProviderBase).  Double-inheriting causes
// a diamond ambiguity that breaks moc.

class FrameProvider : public QQuickImageProvider
{
    Q_OBJECT
    Q_PROPERTY(int frameCounter READ frameCounter NOTIFY frameCounterChanged)

public:
    explicit FrameProvider();

    // QQuickImageProvider interface — called on the QML render thread
    QImage requestImage(const QString& id, QSize* size, const QSize& requestedSize) override;

    int frameCounter() const { return m_frameCounter.load(std::memory_order_relaxed); }

public slots:
    // Connected to SharedMemoryReader::frameUpdated via Qt::QueuedConnection
    void updateFrame(const QImage& image);

signals:
    void frameCounterChanged();

private:
    mutable QMutex m_mutex;
    QImage         m_latest;
    std::atomic<int> m_frameCounter{0};
};
