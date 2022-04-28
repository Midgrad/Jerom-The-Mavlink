#include "mavlink_mission_download.h"

#include <common/mavlink.h>

#include <QDebug>

#include "mavlink_mission_traits.h"

using namespace md::domain;

MavlinkMissionDownload::MavlinkMissionDownload(MavlinkHandlerContext* context,
                                               IMissionsService* missionsService, QObject* parent) :
    MavlinkMissionBase(context, missionsService, parent)
{
}

void MavlinkMissionDownload::parse(const mavlink_message_t& message, const QVariant& vehicleId)
{
    switch (message.msgid)
    {
    case MAVLINK_MSG_ID_MISSION_COUNT:
        return this->processMissionCount(message, vehicleId);
    case MAVLINK_MSG_ID_MISSION_ITEM:
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        return this->processMissionItem(message, vehicleId);
    default:
        break;
    }
}

void MavlinkMissionDownload::processMissionCount(const mavlink_message_t& message,
                                                 const QVariant& vehicleId)
{
    MissionOperation* operation = m_vehicleOperations.value(vehicleId);
    if (!operation)
        return;

    if (m_operationStates.value(operation, Idle) != WaitingCount)
    {
        qDebug() << vehicleId << "mavlink_mission_count_t ignored";
        return;
    }

    mavlink_mission_count_t mission_count;
    mavlink_msg_mission_count_decode(&message, &mission_count);

    qDebug() << "processMissionCount" << vehicleId << mission_count.count;

    operation->total = mission_count.count;
    m_operationStates[operation] = WaitingItem;

    this->sendMissionItemRequest(vehicleId, operation->progress());
}

void MavlinkMissionDownload::processMissionItem(const mavlink_message_t& message,
                                                const QVariant& vehicleId)
{
    MissionOperation* operation = m_vehicleOperations.value(vehicleId);
    if (!operation)
        return;

    mavlink_mission_item_t item;
    mavlink_msg_mission_item_decode(&message, &item);

    // TODO: target waypoint, not in this handler
    if (m_operationStates.value(operation, Idle) != WaitingItem)
        return;

    qDebug() << "processMissionItem" << vehicleId << item.seq;

    Mission* mission = operation->mission();
    MissionRoute* route = mission->route();

    auto convertor = item.seq ? m_convertorsPool.convertor(item.command) : m_convertorsPool.homeConvertor();
    if (convertor)
    {
        MissionRouteItem* routeItem = route->item(item.seq);
        if (!routeItem)
        {
            routeItem = new MissionRouteItem(&mission::waypoint,
                                             mission::waypoint.shortName); // TODO: type by convertor
            route->addItem(routeItem);
        }

        convertor->toItem(item, routeItem);

        if (item.seq == 0) // HOME item
            m_convertorsPool.setHomeAltitude(routeItem->position().altitude);
    }
    else
    {
        qWarning() << "Unhandled mission item type" << item.command;
    }

    // Update mission progress
    operation->progress = item.seq + 1;

    if (operation->isComplete())
    {
        // Clear extra items in route

        while (route->count() > operation->total)
        {
            route->removeItem(route->item(route->count() - 1));
        }

        this->sendAck(vehicleId, MAV_MISSION_ACCEPTED);
        m_missionsService->saveMission(mission);
        m_missionsService->endOperation(operation, MissionOperation::Succeeded);
    }
    else
    {
        // Request next waypoint
        this->sendMissionItemRequest(vehicleId, operation->progress());
    }
}

void MavlinkMissionDownload::sendMissionRequest(const QVariant& vehicleId)
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

void MavlinkMissionDownload::sendMissionItemRequest(const QVariant& vehicleId, int index)
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

void MavlinkMissionDownload::onOperationStarted(MissionOperation* operation)
{
    if (operation->type() != MissionOperation::Download)
        return;

    if (!operation->mission()->route())
        return;

    QVariant vehicleId = operation->mission()->vehicleId();
    m_vehicleOperations.insert(vehicleId, operation);
    m_operationStates[operation] = WaitingCount;

    this->sendMissionRequest(vehicleId);
}
