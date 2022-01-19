#ifndef MAVLINK_PROTOCOL_H
#define MAVLINK_PROTOCOL_H

#include "i_communication_protocol.h"

#include <QVector>

#include "i_mavlink_handlers_factory.h"

namespace md::data_source
{
class MavlinkProtocol : public ICommunicationProtocol
{
    Q_OBJECT
public:
    MavlinkProtocol(domain::IMavlinkHandlerFactory* factory, QObject* parent = nullptr);
    ~MavlinkProtocol();

public slots:
    void receiveData(const QByteArray& data) override;

private slots:
    void sendMessage(const mavlink_message_t& message);

private:
    void parseMessage(const QByteArray& data);

    domain::MavlinkHandlerContext m_context;
    QVector<domain::IMavlinkHandler*> const m_handlers;
};
} // namespace md::data_source

#endif // MAVLINK_PROTOCOL_H
