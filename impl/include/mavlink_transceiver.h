#ifndef MAVLINK_TRANSCEIVER_H
#define MAVLINK_TRANSCEIVER_H

#include "i_mavlink_transceiver.h"

#include <QMap>
#include <QVector>

#include <memory>

#include "i_mavlink_handlers_factory.h"
#include "jerom_traits.h"
#include "link_configuration.h"

namespace md::domain
{
class MavlinkTransceiver : public IMavlinkTransceiver
{
    Q_OBJECT

public:
    MavlinkTransceiver(data_source::LinkConfiguration* configuration,
                       IMavlinkHandlerFactory* factory, QObject* parent = nullptr);

    const QVector<IMavlinkHandler*>& handlers() const;

public slots:
    void start() override;
    void stop() override;

protected:
    void timerEvent(QTimerEvent* event) override;

private slots:
    void receiveData();
    void send(const mavlink_message_t& message);

private:
    int m_timerId = 0;
    //    static void parseData(const std::string& data, MavlinkTransceiver* transceiver);
    static void parseMessage(const std::string& data, MavlinkTransceiver* transceiver);
    MavlinkHandlerContext m_context;
    data_source::LinkPtrMap const m_links;
    QVector<IMavlinkHandler*> const m_handlers;
    data_source::LinkConfiguration* m_configuration;

public:
};
} // namespace md::domain

#endif // MAVLINK_TRANSCEIVER_H
