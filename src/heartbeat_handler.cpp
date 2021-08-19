#include "heartbeat_handler.h"

#include "i_property_tree.h"
#include "locator.h"
#include <QDebug>
#include <QTimerEvent>

using namespace jerom_mavlink;

HeartbeatHandler::HeartbeatHandler()
{
}

HeartbeatHandler::~HeartbeatHandler()
{
}

bool HeartbeatHandler::canParse(quint32 msgId)
{
    return (msgId == MAVLINK_MSG_ID_HEARTBEAT);
}

void HeartbeatHandler::parseMessage(const mavlink_message_t& message)
{
    this->processHeartbeat(message);
}

void HeartbeatHandler::processHeartbeat(const mavlink_message_t& message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&message, &heartbeat);

    //    qDebug() << heartbeat.type;
    //    qDebug() << heartbeat.system_status;
    //    qDebug() << heartbeat.base_mode;
}
