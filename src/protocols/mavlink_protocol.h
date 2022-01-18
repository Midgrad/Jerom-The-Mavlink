#ifndef MAVLINK_PROTOCOL_H
#define MAVLINK_PROTOCOL_H

#include "i_communication_protocol.h"

#include <QMap>
#include <QVector>

#include <memory>

#include "communication/tranceivers/link_transceiver_threaded.h"
#include "i_mavlink_handlers_factory.h"
#include "link_traits.h"

namespace md::data_source
{
class MavlinkProtocol : public domain::ICommunicationProtocol
{
    Q_OBJECT
public:
    explicit MavlinkProtocol(domain::IMavlinkHandlerFactory* factory, QObject* parent = nullptr);
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
