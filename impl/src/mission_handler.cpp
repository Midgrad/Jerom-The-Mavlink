#include "mission_handler.h"

#include <QDebug>
#include <QThread>

#include "mavlink_mission_factory.h"
#include "mavlink_mission_waypoint.h"
#include "mavlink_protocol_helpers.h"
#include "mavlink_tmi.h"

using namespace md::domain;

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

    // TODO: refactor to other class
    connect(missionsService, &IMissionsService::missionAdded, this,
            &MissionHandler::subscribeMission, Qt::DirectConnection);
    connect(missionsService, &IMissionsService::missionRemoved, this,
            &MissionHandler::unsubscribeMission, Qt::DirectConnection);
    for (Mission* mission : missionsService->missions())
    {
        this->subscribeMission(mission);
    }
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

    mavlink_mission_item_int_t item;
    mavlink_msg_mission_item_int_decode(&message, &item);

    qDebug() << "processMissionItem" << node << item.seq;

    Route* route = mission->route();
    if (!route)
    {
        MavlinkMissionFactory factory;
        route = factory.createRouteForMission(mission);
    }

    Waypoint* waypoint = nullptr;
    if (item.seq < route->count())
    {
        waypoint = route->waypoint(item.seq);
    }
    else
    {
        MavlinkMissionFactory factory;
        waypoint = factory.createWaypointForRoute(route, &mavlink_mission::waypoint);
    }

    MavlinkMissionWaypoint missionWaypoint(waypoint);
    missionWaypoint.fillFromMissionItem(item);

    mission->setProgress(item.seq + 1);

    if (mission->isComplete())
    {
        m_downloadingMissions.remove(node);
        this->sendAck(node, MAV_MISSION_ACCEPTED);
    }
    else
    {
        // Request next waypoint
        this->sendMissionItemRequest(node, mission->progress());
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
        mission->setTotal(mission_count.count);
        mission->setProgress(0);

        this->sendMissionItemRequest(node, mission->progress());
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

void MissionHandler::subscribeMission(Mission* mission)
{
    if (mission->type() != mavlink_mission::missionType)
        return;

    connect(mission, &Mission::download, this, [this, mission]() {
        m_downloadingMissions[mission->vehicle()] = mission;
        this->sendMissionRequest(mission->vehicle());
    });

    connect(mission, &Mission::cancel, this, [this, mission]() {
        QString node = m_downloadingMissions.key(mission);
        if (node.length())
            m_downloadingMissions.remove(node);

        node = m_uploadingMissions.key(mission);
        if (node.length())
            m_uploadingMissions.remove(node);

        // TODO: for downloading only
        mission->setTotal(mission->progress());
    });
}

void MissionHandler::unsubscribeMission(Mission* mission)
{
    QString node = m_downloadingMissions.key(mission);
    if (node.length())
        m_downloadingMissions.remove(node);

    disconnect(mission, nullptr, this, nullptr);
}
