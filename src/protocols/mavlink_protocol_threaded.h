#ifndef MAVLINK_PROTOCOL_THREADED_H
#define MAVLINK_PROTOCOL_THREADED_H

#include "i_communication_protocol.h"

#include <QThread>

namespace md::domain
{
class MavlinkProtocolThreaded : public ICommunicationProtocol
{
    Q_OBJECT

public:
    MavlinkProtocolThreaded(ICommunicationProtocol* worker, QObject* parent = nullptr);
    ~MavlinkProtocolThreaded() override;

public slots:
    void start() override;
    void stop() override;

private:
    ICommunicationProtocol* const m_worker;
    QThread* const m_thread;
};
} // namespace md::domain

#endif // MAVLINK_PROTOCOL_THREADED_H
