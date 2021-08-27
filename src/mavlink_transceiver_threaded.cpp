#include "mavlink_transceiver_threaded.h"

#include <QDebug>

namespace
{
constexpr int timeout = 500;
constexpr char threadName[] = "mavlink_transciever";
} // namespace

using namespace jerom_mavlink::domain;

MavlinkTranscieverThreaded::MavlinkTranscieverThreaded(IMavlinkTransciever* worker,
                                                       QObject* parent) :
    IMavlinkTransciever(parent),
    m_worker(worker),
    m_thread(new QThread(this))
{
    Q_ASSERT(worker);

    m_thread->setObjectName(::threadName);
    worker->moveToThread(m_thread);

    QObject::connect(m_thread, &QThread::finished, m_thread, &QThread::deleteLater);
    QObject::connect(m_worker, &IMavlinkTransciever::finished, m_thread, &QThread::quit);
    QObject::connect(m_worker, &IMavlinkTransciever::finished, m_worker, &QObject::deleteLater);
    QObject::connect(m_worker, &IMavlinkTransciever::finished, this, &IMavlinkTransciever::finished);
}

MavlinkTranscieverThreaded::~MavlinkTranscieverThreaded()
{
    m_thread->terminate();
    if (!m_thread->wait(::timeout))
        qCritical() << "Thread" << m_thread->objectName() << "is blocked!";
}

void MavlinkTranscieverThreaded::start()
{
    m_thread->start();

    QMetaObject::invokeMethod(m_worker, "start", Qt::QueuedConnection);
}

void MavlinkTranscieverThreaded::stop()
{
    QMetaObject::invokeMethod(m_worker, "stop", Qt::QueuedConnection);
}
