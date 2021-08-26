#ifndef MAVLINK_TRANSCIEVER_H
#define MAVLINK_TRANSCIEVER_H

#include "i_mavlink_transceiver.h"

#include <QMap>
#include <QVector>

// TODO: i_link
#include "i_mavlink_handler.h"
#include "link_factory.h"

namespace jerom_mavlink::domain
{
class MavlinkTransciever : public IMavlinkTransciever
{
    Q_OBJECT

public:
    MavlinkTransciever(const QMap<QString, loodsman::LinkPtr>& links,
                       const QVector<IMavlinkHandler*>& handlers, QObject* parent = nullptr);

public slots:
    void start() override;
    void stop() override;

protected:
    void timerEvent(QTimerEvent* event) override;

private slots:
    void receiveData();
    void parseMessage(const QByteArray& data);
    void send(const QByteArray& data);

private:
    int m_timerId = 0;
    QMap<QString, loodsman::LinkPtr> const m_links;
    QVector<IMavlinkHandler*> const m_handlers;
};
} // namespace jerom_mavlink::domain

#endif // MAVLINK_TRANSCIEVER_H
