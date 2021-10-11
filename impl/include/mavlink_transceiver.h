#ifndef MAVLINK_TRANSCEIVER_H
#define MAVLINK_TRANSCEIVER_H

#include "i_mavlink_transceiver.h"

#include <QMap>
#include <QVector>

#include <memory>

#include "i_mavlink_handlers_factory.h"
#include "jerom_traits.h"

namespace md::domain
{
class MavlinkTransceiver : public IMavlinkTransceiver
{
    Q_OBJECT

public:
    MavlinkTransceiver(const data_source::LinkPtrMap& links, IMavlinkHandlerFactory* factory,
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
    data_source::LinkPtrMap const m_links;
    QVector<IMavlinkHandler*> const m_handlers;
};
} // namespace md::domain

#endif // MAVLINK_TRANSCEIVER_H
