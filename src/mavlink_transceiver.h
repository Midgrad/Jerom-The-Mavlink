#ifndef MAVLINK_TRANSCIEVER_H
#define MAVLINK_TRANSCIEVER_H

#include "i_mavlink_transceiver.h"

// TODO: ilink
#include "link_factory.h"

#include <QJsonArray>
#include <QMap>

namespace jerom_mavlink::domain
{
class MavlinkTransciever : public IMavlinkTransciever
{
    Q_OBJECT

public:
    MavlinkTransciever(const QJsonArray& config, QObject* parent = nullptr);

public slots:
    void send(const QString& path, const QJsonObject& properties) override;

private:
    QMap<QString, loodsman::link_ptr> m_links;
};
} // namespace jerom_mavlink::domain

#endif // MAVLINK_TRANSCIEVER_H
