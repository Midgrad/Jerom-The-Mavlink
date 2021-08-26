#ifndef MAVLINK_TRANSCIEVER_THREADED_H
#define MAVLINK_TRANSCIEVER_THREADED_H

#include "i_mavlink_transceiver.h"

namespace jerom_mavlink::domain
{
class MavlinkTranscieverThreaded : public IMavlinkTransciever
{
    Q_OBJECT

public:
    MavlinkTranscieverThreaded(QObject* parent = nullptr);

public slots:
    void send(const QString& path, const QJsonObject& properties) override;
};
} // namespace jerom_mavlink::domain

#endif // MAVLINK_TRANSCIEVER_THREADED_H
