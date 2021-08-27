#include "heartbeat_handler.h"

#include <QDebug>

using namespace jerom_mavlink::domain;

namespace
{
std::string decodeMavType(uint8_t type)
{
    switch (type)
    {
    case MAV_TYPE_FIXED_WING:
    case MAV_TYPE_KITE:
    case MAV_TYPE_FLAPPING_WING:
        return "FixedWing";
    case MAV_TYPE_TRICOPTER:
    case MAV_TYPE_QUADROTOR:
    case MAV_TYPE_HEXAROTOR:
    case MAV_TYPE_OCTOROTOR:
        return "Multicopter";
    case MAV_TYPE_COAXIAL:
    case MAV_TYPE_HELICOPTER:
        return "Helicopter";
    case MAV_TYPE_VTOL_DUOROTOR:
    case MAV_TYPE_VTOL_QUADROTOR:
    case MAV_TYPE_VTOL_TILTROTOR:
    case MAV_TYPE_VTOL_RESERVED2:
    case MAV_TYPE_VTOL_RESERVED3:
    case MAV_TYPE_VTOL_RESERVED4:
    case MAV_TYPE_VTOL_RESERVED5:
        return "Vtol";
    case MAV_TYPE_AIRSHIP:
    case MAV_TYPE_FREE_BALLOON:
        return "Airship";
    case MAV_TYPE_GENERIC:
    default:
        return "Vehicle";
    }
}

std::string decodeState(uint8_t state)
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

// TODO: Options is not mutually exclusive, for demo purposes only
std::string decodeMode(uint8_t mode)
{
    if (mode & MAV_MODE_FLAG_DECODE_POSITION_AUTO)
        return "Auto";
    else if (mode & MAV_MODE_FLAG_DECODE_POSITION_GUIDED)
        return "Guided";
    else if (mode & MAV_MODE_FLAG_DECODE_POSITION_STABILIZE)
        return "Stabilize";
    else if (mode & MAV_MODE_FLAG_DECODE_POSITION_MANUAL)
        return "Manual";
    else
        return "Unknown";
}

} // namespace

HeartbeatHandler::HeartbeatHandler(QObject* parent) : IMavlinkHandler(parent)
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

    emit propertiesObtained(
        QStringLiteral("MAV %1").arg(message.sysid),
        QJsonObject({ { "state", QString::fromStdString(decodeState(heartbeat.system_status)) },
                      { "armed", (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) },
                      { "mode", QString::fromStdString(decodeMode(heartbeat.base_mode)) },
                      { "type", QString::fromStdString(decodeMavType(heartbeat.type)) } }));
}
