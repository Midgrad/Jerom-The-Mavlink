#include "mission_handler.h"

#include <QDebug>
#include <QThread>

#include "mavlink_mission_traits.h"
#include "mavlink_protocol_helpers.h"
#include "mavlink_tmi.h"

using namespace md::domain;

namespace
{
const QMap<uint16_t, const WaypointType*> commandTypes = {
    { MAV_CMD_DO_SET_HOME, &mavlink_mission::home },
    { MAV_CMD_NAV_WAYPOINT, &mavlink_mission::waypoint },
    { MAV_CMD_NAV_TAKEOFF, &mavlink_mission::takeoff },
    { MAV_CMD_NAV_LAND, &mavlink_mission::landing },
    { MAV_CMD_NAV_LOITER_TURNS, &mavlink_mission::loiterTurns },
    { MAV_CMD_NAV_LOITER_TO_ALT, &mavlink_mission::loiterAlt }
};
}

MissionHandler::MissionHandler(MavlinkHandlerContext* context, IMissionsService* missionsService,
                               QObject* parent) :
    IMavlinkHandler(context, parent)
{
    connect(m_context->pTree, &IPropertyTree::rootNodesChanged, this,
            [this](const QStringList& nodes) {
                for (const QString& node : nodes)
                {
                    if (!m_obtainedNodes.contains(node))
                        this->sendMissionRequest(node);
                }
            });
    // TODO: common command subsribe
    connect(m_context->pTree, &IPropertyTree::propertiesChanged, this,
            [this](const QString& node, const QVariantMap& properties) {
                if (properties.contains(tmi::setWp))
                {
                    this->m_context->pTree->removeProperties(node, { tmi::setWp });
                    this->sendMissionSetCurrent(node, properties.value(tmi::setWp).toInt());
                }
            });

    connect(missionsService, &IMissionsService::upload, this, &MissionHandler::upload);
    connect(missionsService, &IMissionsService::download, this, &MissionHandler::download);
    connect(missionsService, &IMissionsService::cancel, this, &MissionHandler::cancel);
    connect(missionsService, &IMissionsService::missionRemoved, this, &MissionHandler::cancel);
    connect(this, &MissionHandler::statusUpdate, missionsService, &IMissionsService::updateStatus);
}

MissionHandler::~MissionHandler()
{
}

bool MissionHandler::canParse(quint32 msgId)
{
    return (msgId == MAVLINK_MSG_ID_MISSION_CURRENT) || (msgId == MAVLINK_MSG_ID_MISSION_COUNT) ||
           (msgId == MAVLINK_MSG_ID_MISSION_ITEM_REACHED) ||
           (msgId == MAVLINK_MSG_ID_MISSION_ITEM) || (msgId == MAVLINK_MSG_ID_MISSION_ITEM_INT);
}

void MissionHandler::parseMessage(const mavlink_message_t& message)
{
    switch (message.msgid)
    {
    case MAVLINK_MSG_ID_MISSION_CURRENT:
        return this->processMissionCurrent(message);
    case MAVLINK_MSG_ID_MISSION_COUNT:
        return this->processMissionCount(message);
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
        return this->processMissionReached(message);
    case MAVLINK_MSG_ID_MISSION_ITEM:
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        return this->processMissionItem(message);
    default:
        break;
    }
}

void MissionHandler::sendMissionRequest(const QString& node)
{
    qDebug() << "sendMissionRequest" << node;
    auto mavId = utils::mavIdFromNode(node);
    if (!mavId)
        return;

    mavlink_message_t message;
    mavlink_mission_request_list_t request;

    request.target_system = mavId;
    request.target_component = MAV_COMP_ID_MISSIONPLANNER;

    mavlink_msg_mission_request_list_encode_chan(m_context->systemId, m_context->compId, 0,
                                                 &message,
                                                 &request); // TODO: link channel
    emit sendMessage(message);
}

void MissionHandler::sendMissionItemRequest(const QString& node, int index)
{
    qDebug() << "sendMissionItemRequest" << node << index;
    auto mavId = utils::mavIdFromNode(node);
    if (!mavId)
        return;

    mavlink_message_t message;
    mavlink_mission_request_t request;
    request.target_system = mavId;
    request.target_component = 0;
    request.seq = index;

#ifdef MAVLINK_V2
    request.mission_type = MAV_MISSION_TYPE_MISSION;
#endif

    mavlink_msg_mission_request_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                            &request); // TODO: link channel
    emit sendMessage(message);
}

void MissionHandler::sendAck(const QString& node, MAV_MISSION_RESULT type)
{
    qDebug() << "sendAck" << node << type;
    auto mavId = utils::mavIdFromNode(node);
    if (!mavId)
        return;

    mavlink_message_t message;
    mavlink_mission_ack_t ack;
    ack.target_system = mavId;
    ack.target_component = 0;
    ack.type = type;

#ifdef MAVLINK_V2
    ack.mission_type = MAV_MISSION_TYPE_MISSION;
#endif

    mavlink_msg_mission_ack_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                        &ack); // TODO: link channel
    emit sendMessage(message);
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

void MissionHandler::processMissionItem(const mavlink_message_t& message)
{
    QString node = utils::nodeFromMavId(message.sysid);
    if (node.isEmpty())
        return;

    Mission* mission = m_downloadingMissions.value(node, nullptr);
    if (!mission)
        return;

    mavlink_mission_item_t item;
    mavlink_msg_mission_item_decode(&message, &item);

    qDebug() << "processMissionItem" << node << item.seq;

    Route* route = mission->route();
    Waypoint* waypoint = nullptr;
    if (item.seq < route->count())
    {
        waypoint = route->waypoint(item.seq);
    }
    else
    {
        waypoint = new Waypoint(item.seq ? tr("WPT %1").arg(item.seq) : QObject::tr("HOME"),
                                &mavlink_mission::waypoint);
        route->addWaypoint(waypoint);
    }

    auto type = ::commandTypes.value(item.command, nullptr);
    auto convertor = m_convertors.convertor(type);
    if (type && convertor)
    {
        convertor->itemToWaypoint(item, waypoint);
    }
    else
    {
        qWarning() << "Unhandled mission item type" << item.command;
    }

    MissionStatus status(item.seq + 1, m_statuses[node].total());
    m_statuses[node] = status;
    emit statusUpdate(mission->id(), status);

    if (status.isComplete())
    {
        m_downloadingMissions.remove(node);
        this->sendAck(node, MAV_MISSION_ACCEPTED);
    }
    else
    {
        // Request next waypoint
        this->sendMissionItemRequest(node, status.progress());
    }
}

void MissionHandler::processMissionCurrent(const mavlink_message_t& message)
{
    QString node = utils::nodeFromMavId(message.sysid);
    if (node.isEmpty())
        return;

    mavlink_mission_current_t mission_current;
    mavlink_msg_mission_current_decode(&message, &mission_current);

    m_context->pTree->appendProperties(node, { { tmi::wp, mission_current.seq } });
}

void MissionHandler::processMissionCount(const mavlink_message_t& message)
{
    QString node = utils::nodeFromMavId(message.sysid);
    if (node.isEmpty())
        return;

    mavlink_mission_count_t mission_count;
    mavlink_msg_mission_count_decode(&message, &mission_count);

    if (!m_obtainedNodes.contains(node))
        m_obtainedNodes.append(node);

    m_context->pTree->appendProperties(node, { { tmi::wpCount, mission_count.count } });

    Mission* mission = m_downloadingMissions.value(node, nullptr);
    if (mission)
    {
        MissionStatus status(0, mission_count.count);
        m_statuses[node] = status;

        emit statusUpdate(mission->id(), status);
        this->sendMissionItemRequest(node, status.progress());
    }
}

void MissionHandler::processMissionReached(const mavlink_message_t& message)
{
    QString node = utils::nodeFromMavId(message.sysid);
    if (node.isEmpty())
        return;

    mavlink_mission_item_reached_t reached;
    mavlink_msg_mission_item_reached_decode(&message, &reached);

    // TODO: mark waypoint with reached flag
}

void MissionHandler::upload(Mission* mission)
{
    if (mission->vehicle().isEmpty())
        return;

    m_uploadingMissions[mission->vehicle()] = mission;
    //TODO: this->sendMissionCount(mission->vehicle(), mission->route()->count());
}

void MissionHandler::download(Mission* mission)
{
    if (mission->vehicle().isEmpty())
        return;

    m_downloadingMissions[mission->vehicle()] = mission;
    this->sendMissionRequest(mission->vehicle());
}

void MissionHandler::cancel(Mission* mission)
{
    QString node = m_downloadingMissions.key(mission);
    if (node.length())
    {
        m_downloadingMissions.remove(node);
    }

    node = m_uploadingMissions.key(mission);
    if (node.length())
    {
        m_uploadingMissions.remove(node);
    }

    m_statuses.remove(node);
    emit statusUpdate(mission->id(), MissionStatus());
}
