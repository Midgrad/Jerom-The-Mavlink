#include "heartbeat_handler.h"

#include "i_property_tree.h"
#include "locator.h"
#include <QDebug>
#include <QJsonObject>
#include <QString>
#include <QTimerEvent>

using namespace jerom_mavlink;

namespace
{
std::string decodeState(quint8 state)
{
    switch (state)
    {
    case MAV_STATE_BOOT:
        return "Preparing";
    case MAV_STATE_CALIBRATING:
        return "Calibrating";
    case MAV_STATE_STANDBY:
        return "Standby";
    case MAV_STATE_ACTIVE:
        return "Active";
    case MAV_STATE_CRITICAL:
        return "Critical";
    case MAV_STATE_EMERGENCY:
        return "Emergency";
    case MAV_STATE_POWEROFF:
        return "Poweroff";
    case MAV_STATE_FLIGHT_TERMINATION:
        return "Flight Termination";
    case MAV_STATE_UNINIT:
    default:
        return "Unknown";
    }
}

// TODO: Use bitwise & operators with masks to decode
// Not working properly
std::string decodeMode(quint8 mode)
{
    switch (mode & 126) // 0b01111100 - mask for every mode flag
    {
    case MAV_MODE_FLAG_AUTO_ENABLED:
        return "Auto";
    case MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
        return "Manual";
    case MAV_MODE_FLAG_GUIDED_ENABLED:
        return "Guided";
    case MAV_MODE_FLAG_STABILIZE_ENABLED:
        return "Stabilize";
    case MAV_MODE_FLAG_HIL_ENABLED:
        return "HIL";
    case MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
        return "Custom";
    default:
        return "Unknown";
    }
}

} // namespace

HeartbeatHandler::HeartbeatHandler()
{
    m_pTree = kjarni::domain::Locator::get<kjarni::domain::IPropertyTree>();
    Q_ASSERT(m_pTree);
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

    m_pTree->appendProperties(
        "MAV 23",
        QJsonObject({ { "state", QString::fromStdString(decodeState(heartbeat.system_status)) },
                      { "armed", (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) },
                      { "mode", QString::fromStdString(decodeMode(heartbeat.base_mode)) } }));
}
