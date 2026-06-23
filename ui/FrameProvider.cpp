// ui/FrameProvider.cpp
#include "FrameProvider.h"

FrameProvider::FrameProvider()
    : QQuickImageProvider(QQuickImageProvider::Image)
{}

QImage FrameProvider::requestImage(const QString& /*id*/, QSize* size, const QSize& /*requestedSize*/)
{
    QMutexLocker locker(&m_mutex);
    if (size)
        *size = m_latest.size();
    return m_latest;
}

void FrameProvider::updateFrame(const QImage& image)
{
    {
        QMutexLocker locker(&m_mutex);
        m_latest = image;
    }
    int next = m_frameCounter.fetch_add(1, std::memory_order_relaxed) + 1;
    Q_UNUSED(next)
    emit frameCounterChanged();
}
