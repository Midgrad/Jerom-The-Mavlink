#include "mission_handler.h"

#include <QDebug>

#include "mavlink_protocol_helpers.h"
#include "mavlink_tmi.h"

using namespace md::domain;

MissionHandler::MissionHandler(MavlinkHandlerContext* context, QObject* parent) :
    IMavlinkHandler(context, parent)
{
    // TODO: common command subsribe
    connect(m_context->pTree, &IPropertyTree::propertiesChanged, this,
            [this](const QString& node, const QVariantMap& properties) {
                if (properties.contains(tmi::setWp))
                {
                    this->m_context->pTree->removeProperties(node, { tmi::setWp });
                    this->sendMissionSetCurrent(node, properties.value(tmi::setWp).toInt());
                }
            });
}

MissionHandler::~MissionHandler()
{
}

bool MissionHandler::canParse(quint32 msgId)
{
    return (msgId == MAVLINK_MSG_ID_MISSION_COUNT) || (msgId == MAVLINK_MSG_ID_MISSION_CURRENT);
}

void MissionHandler::parseMessage(const mavlink_message_t& message)
{
    if (message.msgid == MAVLINK_MSG_ID_MISSION_CURRENT)
    {
        this->processMissionCurrent(message);
    }
    if (message.msgid == MAVLINK_MSG_ID_MISSION_COUNT)
    {
        this->processMissionCount(message);
    }
}

void MissionHandler::sendMissionSetCurrent(const QString& node, int waypoint)
{
    qDebug() << "sendMissionSetCurrent" << node << waypoint;
    auto mavId = utils::mavIdFromNode(node);
    if (!mavId)
        return;

    mavlink_message_t message;
    mavlink_mission_set_current_t setCurrent;

    setCurrent.target_system = mavId;
    setCurrent.target_component = MAV_COMP_ID_MISSIONPLANNER;
    setCurrent.seq = waypoint;

    mavlink_msg_mission_set_current_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                                &setCurrent); // TODO: link channel
    emit sendMessage(message);
}

void MissionHandler::processMissionCurrent(const mavlink_message_t& message)
{
    mavlink_mission_current_t mission_current;
    mavlink_msg_mission_current_decode(&message, &mission_current);

    m_context->pTree->appendProperties(utils::nodeFromMavId(message.sysid),
                                       { { tmi::wpCount, 1000 }, // FIXME: request mission count
                                         { tmi::wp, mission_current.seq } });
}

void MissionHandler::processMissionCount(const mavlink_message_t& message)
{
    mavlink_mission_count_t mission_count;
    mavlink_msg_mission_count_decode(&message, &mission_count);

    m_context->pTree->appendProperties(utils::nodeFromMavId(message.sysid),
                                       { { tmi::wpCount, mission_count.count } });
}
