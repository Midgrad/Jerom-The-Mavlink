#ifndef MAVLINK_PROTOCOL_THREADED_H
#define MAVLINK_PROTOCOL_THREADED_H

#include "i_communication_protocol.h"

#include <QThread>

namespace md::data_source
{
class MavlinkProtocolThreaded : public domain::ICommunicationProtocol
{
    Q_OBJECT

public:
    MavlinkProtocolThreaded(ICommunicationProtocol* worker, QObject* parent = nullptr);
    ~MavlinkProtocolThreaded() override;

public slots:
    void receiveData(const QByteArray& data) override;

signals:
    void dataReceived(QByteArray data);

private:
    ICommunicationProtocol* const m_worker;
    QThread* const m_thread;
};
} // namespace md::data_source

#endif // MAVLINK_PROTOCOL_THREADED_H
