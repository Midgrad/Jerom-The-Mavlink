#include "mavlink_transceiver_threaded.h"

using namespace jerom_mavlink::domain;

MavlinkTranscieverThreaded::MavlinkTranscieverThreaded(QObject* parent) :
    IMavlinkTransciever(parent)
{
}

void MavlinkTranscieverThreaded::send(const QString& path, const QJsonObject& properties)
{
}
