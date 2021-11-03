#include "mission_handler.h"

#include <QDebug>
#include <QThread>

#include "mavlink_mission_traits.h"
#include "mavlink_protocol_helpers.h"
#include "mavlink_tmi.h"
#include "utils.h"

using namespace md::domain;

MissionHandler::MissionHandler(MavlinkHandlerContext* context,
                               IMissionsRepository* missionsRepository, QObject* parent) :
    IMavlinkHandler(context, parent),
    m_missionsRepository(missionsRepository)
{
    // TODO: mission request & creation

    connect(missionsRepository, &IMissionsRepository::missionAdded, this,
            &MissionHandler::onMissionAdded);
    connect(missionsRepository, &IMissionsRepository::missionRemoved, this,
            &MissionHandler::onMissionRemoved);
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

    qDebug() << "MAGIC QDEBUG"; // FIXME: locked thread

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

void MissionHandler::sendMissionSetCurrent(const QVariant& vehicleId, int waypoint)
{
    qDebug() << "sendMissionSetCurrent" << vehicleId << waypoint;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
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

    qDebug() << "processMissionItem" << vehicleId << item.seq;

    // Get or create route
    Route* route = mission->route();
    if (!route)
    {
        mission->assignRoute(
            new Route(&mavlink_mission::routeType, tr("%1 route").arg(mission->name())));
        m_missionsRepository->saveMission(mission);
        route = mission->route();
    }

    Waypoint* waypoint = nullptr;
    if (item.seq < route->count())
    {
        waypoint = route->waypoint(item.seq);
    }
    else
    {
        waypoint = new Waypoint(&mavlink_mission::waypoint,
                                item.seq ? tr("WPT %1").arg(item.seq) : QObject::tr("HOME"));
        route->addWaypoint(waypoint);
    }

    auto convertor = m_convertors.convertor(item.command);
    if (convertor)
    {
        convertor->itemToWaypoint(item, waypoint);
        waypoint->setConfirmed(true);
    }
    else
    {
        qWarning() << "Unhandled mission item type" << item.command;
        waypoint->setConfirmed(false);
    }

    // Update mission progress
    mission->operation()->setProgress(item.seq + 1);

    if (mission->operation()->isComplete())
    {
        this->sendAck(vehicleId, MAV_MISSION_ACCEPTED);
        m_missionsRepository->saveMission(mission);
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

    Route* route = mission->route();
    if (!route)
        return;

    mavlink_mission_current_t mission_current;
    mavlink_msg_mission_current_decode(&message, &mission_current);

    route->setCurrentWaypointIndex(mission_current.seq);
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

    Route* route = mission->route();
    if (!route)
        return;

    mavlink_mission_item_reached_t reached;
    mavlink_msg_mission_item_reached_decode(&message, &reached);

    route->waypoint(reached.seq)->setReached(true);
}

void MissionHandler::onVehicleObtained(Vehicle* vehicle)
{
    // Check we already have mission
    Mission* mission = m_missionsRepository->missionForVehicle(vehicle->id());
    if (mission)
        return;

    // Autocrete mission for new vehicle
    mission = new Mission(&mavlink_mission::missionType, tr("%1 mission").arg(vehicle->name()),
                          vehicle->id());
    m_missionsRepository->saveMission(mission);

    // Automaticaly download mission TODO: to settings
    this->download(mission);
}

void MissionHandler::onMissionAdded(Mission* mission)
{
    m_missionStates[mission] = Idle;
    m_vehicleMissions.insert(mission->vehicleId(), mission);

    connect(mission->operation(), &MissionOperation::upload, this, [this, mission]() {
        this->upload(mission);
    });
    connect(mission->operation(), &MissionOperation::download, this, [this, mission]() {
        this->download(mission);
    });
    connect(mission->operation(), &MissionOperation::cancel, this, [this, mission]() {
        this->cancel(mission);
    });

    // TODO: Immutable
    auto func = [this, mission](Route* route) {
        connect(route, &Route::switchWaypoint, this, [this, mission](int index) {
            this->sendMissionSetCurrent(mission->vehicleId(), index);
            this->cancel(mission);
        });
    };

    if (mission->route())
        func(mission->route());

    connect(mission, &Mission::routeChanged, this, [this, mission, func]() {
        func(mission->route());
    });
}

void MissionHandler::onMissionRemoved(Mission* mission)
{
    m_missionStates.remove(mission);
    m_vehicleMissions.remove(mission->vehicleId());

    disconnect(mission, nullptr, this, nullptr);

    this->cancel(mission);
}

void MissionHandler::upload(Mission* mission)
{
    int count = mission->route() ? mission->route()->count() : 0;
    if (!count)
        return;

    mission->operation()->startUpload(count);
    //TODO: this->sendMissionCount(mission->vehicle(), count);
    m_missionStates[mission] = WaitingRequest;
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
