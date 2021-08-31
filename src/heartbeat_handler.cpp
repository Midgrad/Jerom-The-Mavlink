#include "heartbeat_handler.h"

#include <QDebug>
#include <QJsonArray>

#include "common_tmi.h"
#include "mavlink_protocol_helpers.h"
#include "mode_helper_factory.h"

using namespace md::domain;

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
} // namespace

HeartbeatHandler::HeartbeatHandler(MavlinkHandlerContext* context, QObject* parent) :
    IMavlinkHandler(context, parent)
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

//void HeartbeatHandler::onPropertiesChanged(const QString& path, const QJsonObject& properties)
//{
//    if (properties.contains(tmi::setMode))
//        this->sendMode(path, QJsonValue(properties).toString());
//}

void HeartbeatHandler::sendMode(const QString& node, const QString& mode)
{
    qDebug() << "setMode" << node << mode;
    auto mavId = utils::mavIdFromNode(node);

    auto modeHelper = m_modeHelpers.value(mavId);
    if (!modeHelper)
        return;

    mavlink_message_t message;
    mavlink_set_mode_t setMode;

    setMode.target_system = mavId;
    setMode.base_mode = m_baseModes.value(mavId, 0);
    setMode.custom_mode = modeHelper->modeToCustomMode(mode);

    mavlink_msg_set_mode_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                     &setMode); // TODO: link channel
    emit sendMessage(message);
}

void HeartbeatHandler::processHeartbeat(const mavlink_message_t& message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&message, &heartbeat);

    m_baseModes[message.sysid] = heartbeat.base_mode;

    QVariantMap properties(
        { { tmi::state, QString::fromStdString(::decodeState(heartbeat.system_status)) },
          { tmi::armed, (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) },
          { tmi::type, QString::fromStdString(::decodeMavType(heartbeat.type)) } });

    if (!m_modeHelpers.contains(message.sysid))
        m_modeHelpers.insert(message.sysid,
                             QSharedPointer<data_source::IModeHelper>(
                                 data_source::ModeHelperFactory::create(heartbeat.autopilot,
                                                                        heartbeat.type)));

    auto modeHelper = m_modeHelpers.value(message.sysid);
    if (modeHelper)
    {
        properties.insert(tmi::modes, QJsonArray::fromStringList(modeHelper->availableModes()));
        properties.insert(tmi::mode, modeHelper->customModeToMode(heartbeat.custom_mode));
    }

    m_context->pTree->appendProperties(utils::nodeFromMavId(message.sysid), properties);
}
