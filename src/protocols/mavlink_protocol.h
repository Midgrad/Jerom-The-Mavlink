#ifndef MAVLINK_PROTOCOL_H
#define MAVLINK_PROTOCOL_H

#include "i_communication_protocol.h"

#include <QMap>
#include <QVector>

#include <memory>

#include "i_mavlink_handlers_factory.h"
#include "link_traits.h"
#include "link_transceiver_threaded.h"

namespace md::data_source
{
class MavlinkProtocol : public domain::ICommunicationProtocol
{
    Q_OBJECT
public:
    explicit MavlinkProtocol(domain::IMavlinkHandlerFactory* factory, QObject* parent = nullptr);

    //public slots:
    //    void receiveData(const QByteArray& data);
    //        void start() override;
    //        void stop() override;

private slots:
    void receiveData(const QByteArray& data) override;
    void sendMessage(const mavlink_message_t& message);

private:
    void parseMessage(const QByteArray& data);

    //    int m_timerId = 0;
    domain::MavlinkHandlerContext m_context;
    QVector<domain::IMavlinkHandler*> const m_handlers;
};
} // namespace md::data_source

#endif // MAVLINK_PROTOCOL_H
