#include "mavlink_protocol_threaded.h"

#include <QDebug>

namespace
{
constexpr int timeout = 500;
constexpr char threadName[] = "mavlink_transceiver";
} // namespace

using namespace md::data_source;

MavlinkProtocolThreaded::MavlinkProtocolThreaded(ICommunicationProtocol* worker, QObject* parent) :
    ICommunicationProtocol(parent),
    m_worker(worker),
    m_thread(new QThread(this))
{
    Q_ASSERT(worker);

    m_thread->setObjectName(::threadName);
    worker->moveToThread(m_thread);

    QObject::connect(m_thread, &QThread::finished, m_thread, &QThread::deleteLater);
    QObject::connect(m_worker, &ICommunicationProtocol::finished, m_thread, &QThread::quit);
    QObject::connect(m_worker, &ICommunicationProtocol::finished, m_worker, &QObject::deleteLater);
    //    QObject::connect(m_worker, &IMavlinkTransceiver::finished, this, &IMavlinkTransceiver::finished);

    QObject::connect(this, &MavlinkProtocolThreaded::resendData, m_worker,
                     &ICommunicationProtocol::receiveData);

    QObject::connect(m_worker, &ICommunicationProtocol::sendData, this,
                     &ICommunicationProtocol::sendData);

    m_thread->start();
}

MavlinkProtocolThreaded::~MavlinkProtocolThreaded()
{
    m_thread->quit();

    if (!m_thread->wait(::timeout))
    {
        qCritical() << "Thread" << m_thread->objectName() << "is blocked!";
        qCritical() << "Forcing to terminate...";
        m_thread->terminate();
    }
}

void MavlinkProtocolThreaded::receiveData(const QByteArray& data)
{
    //    qDebug() << "receiveData";
    emit resendData(data);
}

//void MavlinkProtocolThreaded::start()
//{
//    m_thread->start();
//
//    //    QMetaObject::invokeMethod(m_worker, "start", Qt::QueuedConnection);
//}
//
//void MavlinkProtocolThreaded::stop()
//{
//    //    QMetaObject::invokeMethod(m_worker, "stop", Qt::QueuedConnection);
//}
