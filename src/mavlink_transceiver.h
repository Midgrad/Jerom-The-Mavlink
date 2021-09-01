#ifndef MAVLINK_TRANSCEIVER_H
#define MAVLINK_TRANSCEIVER_H

#include "i_mavlink_transceiver.h"

#include <QMap>
#include <QVector>

#include <memory>

// TODO: i_link
#include "i_link_factory.h"
#include "i_mavlink_handlers_factory.h"
#include "link_configuration.h"

namespace md::domain
{
class MavlinkTransceiver : public IMavlinkTransceiver
{
    Q_OBJECT

public:
    MavlinkTransceiver(const QMap<QString, LinkPtr>& links, IMavlinkHandlerFactory* factory,
                       QObject* parent = nullptr);

public slots:
    void start() override;
    void stop() override;

protected:
    void timerEvent(QTimerEvent* event) override;

private slots:
    void receiveData();
    void parseMessage(const QByteArray& data);
    void send(const mavlink_message_t& message);

private:
    int m_timerId = 0;
    MavlinkHandlerContext m_context;
    QMap<QString, LinkPtr> const m_links;
    QVector<IMavlinkHandler*> const m_handlers;
};
} // namespace md::domain

#endif // MAVLINK_TRANSCEIVER_H
