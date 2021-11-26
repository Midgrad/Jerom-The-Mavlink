#include "mavlink_transceiver_threaded.h"

#include <QDebug>

namespace
{
constexpr int timeout = 500;
constexpr char threadName[] = "mavlink_transceiver";
} // namespace

using namespace md::domain;

MavlinkTranscieverThreaded::MavlinkTranscieverThreaded(IMavlinkTransceiver* worker,
                                                       QObject* parent) :
    IMavlinkTransceiver(parent),
    m_worker(worker),
    m_thread(new QThread(this))
{
    Q_ASSERT(worker);

    m_thread->setObjectName(::threadName);
    worker->moveToThread(m_thread);

    QObject::connect(m_thread, &QThread::finished, m_thread, &QThread::deleteLater);
    QObject::connect(m_worker, &IMavlinkTransceiver::finished, m_thread, &QThread::quit);
    QObject::connect(m_worker, &IMavlinkTransceiver::finished, m_worker, &QObject::deleteLater);
    //    QObject::connect(m_worker, &IMavlinkTransceiver::finished, this, &IMavlinkTransceiver::finished);
}

MavlinkTranscieverThreaded::~MavlinkTranscieverThreaded()
{
    //    qDebug() << "Destructing MavlinkTranscieverThreaded, waiting for processes to be killed";
    m_thread->quit();
    //    qDebug() << "Exiting MT thread...";

    if (!m_thread->wait(::timeout))
    {
        qCritical() << "Thread" << m_thread->objectName() << "is blocked!";
        qCritical() << "Forcing to terminate...";
        m_thread->terminate();
    }
    //    qDebug() << "Bye, MavlinkTranscieverThreaded!";
}

void MavlinkTranscieverThreaded::start()
{
    m_thread->start();

    QMetaObject::invokeMethod(m_worker, "start", Qt::QueuedConnection);
}

void MavlinkTranscieverThreaded::stop()
{
    //    qDebug() << "Emitting stop to MT";
    QMetaObject::invokeMethod(m_worker, "stop", Qt::QueuedConnection);
}
