#include "mission_handler.h"

#include <QDebug>
#include <QThread>

#include "mavlink_mission_traits.h"
#include "mavlink_protocol_helpers.h"
#include "mavlink_tmi.h"
#include "utils.h"

using namespace md::domain;

MissionHandler::MissionHandler(MavlinkHandlerContext* context,
                               IMissionsService* missionsService, QObject* parent) :
    IMavlinkHandler(context, parent),
    m_missionsService(missionsService)
{
    // TODO: mission request & creation

    connect(missionsService, &IMissionsService::missionAdded, this,
            &MissionHandler::onMissionAdded);
    connect(missionsService, &IMissionsService::missionRemoved, this,
            &MissionHandler::onMissionRemoved);
}

MissionHandler::~MissionHandler()
{
}

bool MissionHandler::canParse(quint32 msgId)
{
    return (msgId == MAVLINK_MSG_ID_MISSION_CURRENT) || (msgId == MAVLINK_MSG_ID_MISSION_COUNT) ||
           (msgId == MAVLINK_MSG_ID_MISSION_ITEM_REACHED) ||
           (msgId == MAVLINK_MSG_ID_MISSION_ACK) || (msgId == MAVLINK_MSG_ID_MISSION_REQUEST) ||
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
    case MAVLINK_MSG_ID_MISSION_ACK:
        return this->processMissionAck(message);
    case MAVLINK_MSG_ID_MISSION_REQUEST:
        return this->processMissionRequest(message);
    default:
        break;
    }
}

void MissionHandler::sendMissionRequest(const QVariant& vehicleId)
{
    qDebug() << "sendMissionRequest" << vehicleId;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
    if (!mavId)
        return;

    mavlink_message_t message;
    mavlink_mission_request_list_t request;

    request.target_system = mavId;
    request.target_component = MAV_COMP_ID_MISSIONPLANNER;

#ifdef MAVLINK_V2
    request.mission_type = MAV_MISSION_TYPE_MISSION;
#endif

    mavlink_msg_mission_request_list_encode_chan(m_context->systemId, m_context->compId, 0,
                                                 &message,
                                                 &request); // TODO: link channel
    emit sendMessage(message);
}

void MissionHandler::sendMissionItemRequest(const QVariant& vehicleId, int index)
{
    qDebug() << "sendMissionItemRequest" << vehicleId << index;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
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

void MissionHandler::sendAck(const QVariant& vehicleId, MAV_MISSION_RESULT type)
{
    qDebug() << "sendAck" << vehicleId << type;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
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

void MissionHandler::sendMissionSetCurrent(const QVariant& vehicleId, int index)
{
    qDebug() << "sendMissionSetCurrent" << vehicleId << index;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
    if (!mavId)
        return;

    mavlink_mission_set_current_t setCurrent;

    setCurrent.target_system = mavId;
    setCurrent.target_component = MAV_COMP_ID_MISSIONPLANNER;
    setCurrent.seq = index;

    mavlink_message_t message;
    mavlink_msg_mission_set_current_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                                &setCurrent); // TODO: link channel
    emit sendMessage(message);
}

void MissionHandler::sendMissionCount(const QVariant& vehicleId, int count)
{
    qDebug() << "sendMissionCount" << vehicleId << count;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
    if (!mavId)
        return;

    mavlink_mission_count_t countItem;

    countItem.target_system = mavId;
    countItem.target_component = MAV_COMP_ID_MISSIONPLANNER;
    countItem.count = count;

#ifdef MAVLINK_V2
    countItem.mission_type = MAV_MISSION_TYPE_MISSION;
#endif

    mavlink_message_t message;
    mavlink_msg_mission_count_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                          &countItem); // TODO: link channel
    emit sendMessage(message);
}

void MissionHandler::sendMissionItem(const QVariant& vehicleId, RouteItem* waypointItem,
                                     int index)
{
    qDebug() << "sendMissionItem" << vehicleId << index;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
    if (!mavId)
        return;

    auto convertor = index ? m_convertors.convertor(waypointItem->type()->id)
                           : m_convertors.homeConvertor();
    if (!convertor)
    {
        qWarning() << "Unhandled waypoint type" << waypointItem->type()->name;
        return;
    }

    mavlink_mission_item_t item;

    item.target_system = mavId;
    item.target_component = MAV_COMP_ID_MISSIONPLANNER;
    item.seq = index;
    item.current = false;     // TODO: send current
    item.autocontinue = true; // TODO: is last

#ifdef MAVLINK_V2
    item.mission_type = MAV_MISSION_TYPE_MISSION;
#endif

    convertor->fromItem(waypointItem, item);

    mavlink_message_t message;
    mavlink_msg_mission_item_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                         &item); // TODO: link channel
    emit sendMessage(message);
}

void MissionHandler::processMissionAck(const mavlink_message_t& message)
{
    QString vehicleId = m_context->vehicleIds.value(message.sysid).toString();
    if (vehicleId.isNull())
        return;

    mavlink_mission_ack_t ack;
    mavlink_msg_mission_ack_decode(&message, &ack);

    if (ack.type == MAV_MISSION_ACCEPTED)
        qDebug() << "Accepted";
    else
        qDebug() << "Denied" << ack.type;

    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    if (m_missionStates.value(mission, Idle) == WaitingAck && mission->route()->count())
    {
        // TODO: confirm received item
        //        if (ack.type == MAV_MISSION_ACCEPTED)
        //            mission->item(mission->count() - 1)->setConfirmed(true);

        mission->operation()->stop();
        m_missionStates[mission] = Idle;
    }
}

void MissionHandler::processMissionRequest(const mavlink_message_t& message)
{
    QString vehicleId = m_context->vehicleIds.value(message.sysid).toString();
    if (vehicleId.isNull())
        return;

    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    if (m_missionStates.value(mission, Idle) != WaitingRequest)
    {
        qDebug() << "Got unexpexcted mission request";
        return;
    }

    mavlink_mission_request_t request;
    mavlink_msg_mission_request_decode(&message, &request);

    // TODO: Mark previous waypoint as confirmed
    //    if (request.seq > 0)
    //    {
    //        WaypointItem* previousItem = mission->item(request.seq - 1);
    //        if (previousItem)
    //            previousItem->setConfirmed(true);
    //    }

    RouteItem* waypointItem = mission->route()->item(request.seq);
    if (!waypointItem)
        return;

    // Update mission progress
    mission->operation()->setProgress(request.seq + 1);

    // Waiting ack after last waypoint send
    if (request.seq == mission->route()->count() - 1)
        m_missionStates[mission] = WaitingAck;

    // Send reqested waypoint
    this->sendMissionItem(vehicleId, waypointItem, request.seq);
}

// Refactor with code to compare and merge two routes
void MissionHandler::processMissionItem(const mavlink_message_t& message)
{
    QString vehicleId = m_context->vehicleIds.value(message.sysid).toString();
    if (vehicleId.isNull())
        return;

    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    if (m_missionStates.value(mission, Idle) != WaitingItem)
    {
        // TODO: target waypoint
        return;
    }

    mavlink_mission_item_t item;
    mavlink_msg_mission_item_decode(&message, &item);

    if (item.seq > 0 && item.seq < mission->route()->count())
    {
        qWarning() << "Route must be cleared before download";
        return;
    }

    qDebug() << "processMissionItem" << vehicleId << item.seq;

    auto convertor = item.seq ? m_convertors.convertor(item.command) : m_convertors.homeConvertor();
    if (convertor)
    {
        RouteItem* wptItem = nullptr;
        // Special case for HOME
        if (item.seq == 0)
        {
            wptItem = mission->route()->homePoint();
        }
        else
        {
            // Get or create route
            Route* route = mission->route()->route();
            if (!route) // TODO: create route while download started
            {
                mission->route()->assignRoute(
                    new Route(&route::mavlinkRouteType, tr("%1 route").arg(mission->name())));
                m_missionsService->saveMission(mission);
                route = mission->route()->route();
            }
            if (convertor->isWaypointItem())
            {
                // waypoint by default
                RouteItem* waypoint = new RouteItem(&route::waypoint);
                wptItem = waypoint;
                route->addItem(waypoint);
            }
            else
            {
                RouteItem* waypoint = nullptr;
                if (route->count())
                {
                    // Add item to last waypoint
                    waypoint = route->items().last();
                }
                else
                {
                    // Or create new waypoint
                    waypoint = new RouteItem(&route::waypoint);
                    route->addItem(waypoint);
                }

                wptItem = new RouteItem(route::waypoint.childTypes.first());
                waypoint->addItem(wptItem);
            }
            wptItem->setName(wptItem->type()->shortName);
        }

        convertor->toItem(item, wptItem);
        // TODO: waypointItem->setConfirmed(true);
    }
    else
    {
        qWarning() << "Unhandled mission item type" << item.command;
    }

    // Update mission progress
    mission->operation()->setProgress(item.seq + 1);

    if (mission->operation()->isComplete())
    {
        this->sendAck(vehicleId, MAV_MISSION_ACCEPTED);
        m_missionsService->saveMission(mission);
        m_missionStates[mission] = Idle;
    }
    else
    {
        // Request next waypoint
        this->sendMissionItemRequest(vehicleId, mission->operation()->progress());
    }
}

void MissionHandler::processMissionCurrent(const mavlink_message_t& message)
{
    QString vehicleId = m_context->vehicleIds.value(message.sysid).toString();
    if (vehicleId.isNull())
        return;

    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    mavlink_mission_current_t mission_current;
    mavlink_msg_mission_current_decode(&message, &mission_current);

    mission->route()->setCurrentItem(mission_current.seq);
}

void MissionHandler::processMissionCount(const mavlink_message_t& message)
{
    QString vehicleId = m_context->vehicleIds.value(message.sysid).toString();
    if (vehicleId.isNull())
        return;

    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    if (m_missionStates.value(mission, Idle) != WaitingCount)
    {
        qDebug() << vehicleId << "mavlink_mission_count_t ignored";
        return;
    }

    mavlink_mission_count_t mission_count;
    mavlink_msg_mission_count_decode(&message, &mission_count);

    qDebug() << "processMissionCount" << vehicleId << mission_count.count;

    // Clear route before downloading new
    mission->route()->route()->clear();

    mission->operation()->startDownload(mission_count.count);
    m_missionStates[mission] = WaitingItem;
    this->sendMissionItemRequest(vehicleId, mission->operation()->progress());
}

void MissionHandler::processMissionReached(const mavlink_message_t& message)
{
    QString vehicleId = m_context->vehicleIds.value(message.sysid).toString();
    if (vehicleId.isNull())
        return;

    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    mavlink_mission_item_reached_t reached;
    mavlink_msg_mission_item_reached_decode(&message, &reached);

    // TODO: mark item as reached
    //    WaypointItem* waypoint = mission->waypoint(reached.seq);
    //    if (waypoint)
    //        waypoint->setReached(true);
}

void MissionHandler::onVehicleObtained(Vehicle* vehicle)
{
    // Check we already have mission
    Mission* mission = m_missionsService->missionForVehicle(vehicle->id());
    if (mission)
        return;

    // Autocrete mission for new vehicle
    mission = new Mission(&mission::mavlinkMissionType, tr("%1 mission").arg(vehicle->name()),
                          vehicle->id());
    m_missionsService->saveMission(mission);

    // Automaticaly download mission TODO: to settings
    this->download(mission);
}

void MissionHandler::onMissionAdded(Mission* mission)
{
    m_missionStates[mission] = Idle;
    m_vehicleMissions.insert(mission->vehicleId(), mission);

    connect(mission->operation(), &MissionOperation::uploadItem, this, [this, mission](int index) {
        this->uploadItem(mission, index);
    });
    connect(mission->operation(), &MissionOperation::upload, this, [this, mission]() {
        this->upload(mission);
    });
    connect(mission->operation(), &MissionOperation::download, this, [this, mission]() {
        this->download(mission);
    });
    connect(mission->operation(), &MissionOperation::cancel, this, [this, mission]() {
        this->cancel(mission);
    });

    connect(mission->route(), &MissionRoute::switchCurrentItem, this, [this, mission](int index) {
        this->sendMissionSetCurrent(mission->vehicleId(), index);
    });
}

void MissionHandler::onMissionRemoved(Mission* mission)
{
    m_missionStates.remove(mission);
    m_vehicleMissions.remove(mission->vehicleId());

    disconnect(mission, nullptr, this, nullptr);

    this->cancel(mission);
}

void MissionHandler::uploadItem(Mission* mission, int index)
{
    RouteItem* wpt = mission->route()->route() ? mission->route()->route()->item(index)
                                                  : nullptr;
    if (!wpt)
        return;

    this->sendMissionItem(mission->vehicleId(), wpt, index);
}

void MissionHandler::upload(Mission* mission)
{
    int count = mission->route()->count();

    mission->operation()->startUpload(count);
    m_missionStates[mission] = WaitingRequest;
    this->sendMissionCount(mission->vehicleId(), count);
}

void MissionHandler::download(Mission* mission)
{
    m_missionStates[mission] = WaitingCount;
    this->sendMissionRequest(mission->vehicleId());
}

void MissionHandler::cancel(Mission* mission)
{
    // TODO: repeat timers, stop timers
    m_missionStates[mission] = Idle;
    mission->operation()->stop();
}
