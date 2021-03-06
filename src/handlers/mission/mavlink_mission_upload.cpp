
#include "mavlink_mission_upload.h"

#include <common/mavlink.h>

#include <QDebug>

#include "mavlink_mission_traits.h"

using namespace md::domain;

MavlinkMissionUpload::MavlinkMissionUpload(MavlinkHandlerContext* context,
                                           IMissionsService* missionsService, QObject* parent) :
    MavlinkMissionBase(context, missionsService, parent)
{
}

void MavlinkMissionUpload::parse(const mavlink_message_t& message, const QVariant& vehicleId)
{
    switch (message.msgid)
    {
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        return this->processMissionRequest(message, vehicleId);
    case MAVLINK_MSG_ID_MISSION_ACK:
        return this->processMissionAck(message, vehicleId);
    default:
        break;
    }
}

void MavlinkMissionUpload::processMissionRequest(const mavlink_message_t& message,
                                                 const QVariant& vehicleId)
{
    MissionOperation* operation = m_vehicleOperations.value(vehicleId);
    if (!operation)
        return;

    if (m_operationStates.value(operation, Idle) != WaitingRequest)
    {
        qDebug() << "Got unexpexcted mission item request";
        return;
    }

    mavlink_mission_request_t request;
    mavlink_msg_mission_request_decode(&message, &request);

    Mission* mission = operation->mission();
    MissionRouteItem* item = mission->route()->item(request.seq);

    if (!item)
    {
        qDebug() << "Invalid mission item request";
        return;
    }

    // Update mission progress
    operation->progress = request.seq + 1;

    // Waiting ack after last waypoint send
    if (request.seq == mission->route()->count() - 1)
        m_operationStates[operation] = WaitingAck;

    // Send reqested waypoint
    this->sendMissionItem(vehicleId, item, request.seq);
}

void MavlinkMissionUpload::processMissionAck(const mavlink_message_t& message,
                                             const QVariant& vehicleId)
{
    MissionOperation* operation = m_vehicleOperations.value(vehicleId);
    if (!operation)
        return;

    if (m_operationStates.value(operation, Idle) != WaitingAck)
        return;

    mavlink_mission_ack_t ack;
    mavlink_msg_mission_ack_decode(&message, &ack);

    MissionOperation::State state;
    if (ack.type == MAV_MISSION_ACCEPTED)
    {
        state = MissionOperation::Succeeded;
    }
    else
    {
        qWarning() << "Vehicle" << vehicleId << " denied upload mission with ack" << ack.type;
        state = MissionOperation::Failed;
    }
    m_missionsService->endOperation(operation, state);
}

void MavlinkMissionUpload::sendMissionCount(const QVariant& vehicleId, int count)
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

void MavlinkMissionUpload::sendMissionItem(const QVariant& vehicleId, MissionRouteItem* routeItem,
                                           int index)
{
    qDebug() << "sendMissionItem" << vehicleId << index;

    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
    if (!mavId)
        return;

    auto convertor = index ? m_convertorsPool.convertor(routeItem->type()->id)
                           : m_convertorsPool.homeConvertor();
    if (!convertor)
    {
        qWarning() << "Unhandled waypoint type" << routeItem->type()->name;
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

    convertor->fromItem(routeItem, item);

    mavlink_message_t message;
    mavlink_msg_mission_item_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                         &item); // TODO: link channel
    emit sendMessage(message);
}

void MavlinkMissionUpload::onOperationStarted(MissionOperation* operation)
{
    if (operation->type() != MissionOperation::Upload)
        return;

    QVariant vehicleId = operation->mission()->vehicleId;
    m_vehicleOperations.insert(vehicleId, operation);
    m_operationStates[operation] = WaitingRequest;
    int count = operation->mission()->route()->count();
    operation->total = count;

    this->sendMissionCount(vehicleId, count);
}
