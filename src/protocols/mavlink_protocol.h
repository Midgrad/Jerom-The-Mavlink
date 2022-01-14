#ifndef MAVLINK_PROTOCOL_H
#define MAVLINK_PROTOCOL_H

#include "i_communication_protocol.h"

#include <QMap>
#include <QVector>

#include <memory>

#include "communication/service/communication_service.h"
#include "i_mavlink_handlers_factory.h"
#include "link_traits.h"
#include "link_transceiver_threaded.h"

namespace md::domain
{
class MavlinkProtocol : public ICommunicationProtocol
{
    Q_OBJECT
public:
    explicit MavlinkProtocol(IMavlinkHandlerFactory* factory, QObject* parent = nullptr);

public slots:
    void start() override;
    void stop() override;

signals:
    void sendData(QByteArray data) override;

private slots:
    void receiveData(const QByteArray& data) override;
    void sendMessage(const mavlink_message_t& message);

private:
    void parseMessage(const QByteArray& data);

    int m_timerId = 0;
    MavlinkHandlerContext m_context;
    QVector<IMavlinkHandler*> const m_handlers;
};
} // namespace md::domain

#endif // MAVLINK_PROTOCOL_H
