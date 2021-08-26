#include "mavlink_transceiver_threaded.h"

namespace
{
const char threadName[] = "mavlink_transciever";
}

using namespace jerom_mavlink::domain;

MavlinkTranscieverThreaded::MavlinkTranscieverThreaded(IMavlinkTransciever* worker,
                                                       QObject* parent) :
    IMavlinkTransciever(parent),
    m_worker(worker),
    m_thread(new QThread(this))
{
    m_thread->setObjectName(::threadName);
    worker->moveToThread(m_thread);

    QObject::connect(m_thread, &QThread::finished, m_thread, &QThread::deleteLater);
}

MavlinkTranscieverThreaded::~MavlinkTranscieverThreaded()
{
    m_worker->deleteLater();
}

void MavlinkTranscieverThreaded::start()
{
    m_thread->start();

    QMetaObject::invokeMethod(m_worker, "start", Qt::QueuedConnection);
}

void MavlinkTranscieverThreaded::stop()
{
    QMetaObject::invokeMethod(m_worker, "stop", Qt::QueuedConnection);

    m_thread->terminate();
    m_thread->wait();
}
