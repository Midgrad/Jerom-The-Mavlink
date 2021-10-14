#include "mission_handler.h"

#include <QDebug>
#include <QThread>

#include "mavlink_mission_traits.h"
#include "mavlink_protocol_helpers.h"
#include "mavlink_tmi.h"

using namespace md::domain;

MissionHandler::MissionHandler(MavlinkHandlerContext* context, IMissionsService* missionsService,
                               IVehiclesService* vehiclesService, ICommandsService* commandsService,
                               QObject* parent) :
    IMavlinkHandler(context, parent),
    m_vehiclesService(vehiclesService)
{
    // TODO: mission request & creation

    connect(missionsService, &IMissionsService::missionAdded, this, &MissionHandler::onMissionAdded);
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
    Vehicle* vehicle = m_vehiclesService->vehicle(node);
    auto mavId = m_context->vehicles.key(vehicle, 0);
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

void MissionHandler::sendMissionItemRequest(const QString& node, int index)
{
    qDebug() << "sendMissionItemRequest" << node << index;
    Vehicle* vehicle = m_vehiclesService->vehicle(node);
    auto mavId = m_context->vehicles.key(vehicle, 0);
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
    Vehicle* vehicle = m_vehiclesService->vehicle(node);
    auto mavId = m_context->vehicles.key(vehicle, 0);
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
    Vehicle* vehicle = m_vehiclesService->vehicle(node);
    auto mavId = m_context->vehicles.key(vehicle, 0);
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
    Vehicle* vehicle = m_context->vehicles.value(message.sysid, nullptr);
    if (!vehicle)
        return;

    Mission* mission = m_vehicleMissions.value(vehicle->id(), nullptr);
    if (!mission)
        return;

    if (m_missionStates.value(mission, Idle) != WaitingItem)
    {
        qDebug() << vehicle->id() << "mavlink_mission_item_t ignored";
        return;
    }

    mavlink_mission_item_t item;
    mavlink_msg_mission_item_decode(&message, &item);

    qDebug() << "processMissionItem" << vehicle->id() << item.seq;

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

    auto convertor = m_convertors.convertor(item.command);
    if (convertor)
    {
        convertor->itemToWaypoint(item, waypoint);
    }
    else
    {
        qWarning() << "Unhandled mission item type" << item.command;
    }

    mission->updateStatusProgress(item.seq + 1);
    if (mission->status().isComplete())
    {
        this->sendAck(vehicle->id(), MAV_MISSION_ACCEPTED);
        m_missionStates[mission] = Idle;
    }
    else
    {
        // Request next waypoint
        this->sendMissionItemRequest(vehicle->id(), mission->status().progress());
    }
}

void MissionHandler::processMissionCurrent(const mavlink_message_t& message)
{
    Vehicle* vehicle = m_context->vehicles.value(message.sysid, nullptr);
    if (!vehicle)
        return;

    Mission* mission = m_vehicleMissions.value(vehicle->id(), nullptr);
    if (!mission)
        return;

    mavlink_mission_current_t mission_current;
    mavlink_msg_mission_current_decode(&message, &mission_current);

    mission->setCurrentWaypoint(mission_current.seq);
}

void MissionHandler::processMissionCount(const mavlink_message_t& message)
{
    Vehicle* vehicle = m_context->vehicles.value(message.sysid, nullptr);
    if (!vehicle)
        return;

    Mission* mission = m_vehicleMissions.value(vehicle->id(), nullptr);
    if (!mission)
        return;

    if (m_missionStates.value(mission, Idle) != WaitingCount)
    {
        qDebug() << vehicle->id() << "mavlink_mission_count_t ignored";
        return;
    }

    mavlink_mission_count_t mission_count;
    mavlink_msg_mission_count_decode(&message, &mission_count);

    qDebug() << "processMissionCount" << vehicle->id() << mission_count.count;

    mission->updateStatus(MissionStatus::Downloading, 0, mission_count.count);
    m_missionStates[mission] = WaitingItem;
    this->sendMissionItemRequest(vehicle->id(), mission->status().progress());
}

void MissionHandler::processMissionReached(const mavlink_message_t& message)
{
    Vehicle* vehicle = m_context->vehicles.value(message.sysid, nullptr);
    if (!vehicle)
        return;

    mavlink_mission_item_reached_t reached;
    mavlink_msg_mission_item_reached_decode(&message, &reached);

    // TODO: mark waypoint with reached flag
}

void MissionHandler::onMissionAdded(Mission* mission)
{
    m_missionStates[mission] = Idle;
    m_vehicleMissions.insert(mission->vehicle(), mission);

    connect(mission, &Mission::upload, this, [this, mission]() {
        this->upload(mission);
    });
    connect(mission, &Mission::download, this, [this, mission]() {
        this->download(mission);
    });
    connect(mission, &Mission::cancel, this, [this, mission]() {
        this->cancel(mission);
    });
    connect(mission, &Mission::switchWaypoint, this, [this, mission](int index) {
        this->sendMissionSetCurrent(mission->vehicle(), index);
        this->cancel(mission);
    });
}

void MissionHandler::onMissionRemoved(Mission* mission)
{
    m_missionStates.remove(mission);
    m_vehicleMissions.remove(mission->vehicle());

    disconnect(mission, nullptr, this, nullptr);

    this->cancel(mission);
}

void MissionHandler::upload(Mission* mission)
{
    int count = mission->route()->count();
    mission->updateStatus(MissionStatus::Uploading, 0, count);
    //TODO: this->sendMissionCount(mission->vehicle(), count);
    m_missionStates[mission] = WaitingRequest;
}

void MissionHandler::download(Mission* mission)
{
    mission->updateStatus(MissionStatus::Downloading, 0, 0);
    this->sendMissionRequest(mission->vehicle());
    m_missionStates[mission] = WaitingCount;
}

void MissionHandler::cancel(Mission* mission)
{
    // TODO: repeat timers, stop timers
    mission->updateStatus(MissionStatus::NotActual, 0, 0);
    m_missionStates[mission] = Idle;
}
