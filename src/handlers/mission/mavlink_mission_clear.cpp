#include "mavlink_mission_clear.h"

#include <common/mavlink.h>

#include <QDebug>

using namespace md::domain;

MavlinkMissionClear::MavlinkMissionClear(MavlinkHandlerContext* context,
                                         IMissionsService* missionsService, QObject* parent) :
    MavlinkMissionBase(context, missionsService, parent)
{
}

void MavlinkMissionClear::parse(const mavlink_message_t& message, const QVariant& vehicleId)
{
    if (message.msgid == MAVLINK_MSG_ID_MISSION_ACK)
    {
        return this->processMissionAck(message, vehicleId);
    }
}

void MavlinkMissionClear::processMissionAck(const mavlink_message_t& message,
                                            const QVariant& vehicleId)
{
    MissionOperation* operation = m_vehicleOperations.value(vehicleId);
    if (!operation)
        return;

    if (m_operationStates.value(operation, Idle) != WaitingAck)
        return;

    mavlink_mission_ack_t ack;
    mavlink_msg_mission_ack_decode(&message, &ack);

    // clear mission in db after vehicle accepted
    if (ack.type == MAV_MISSION_ACCEPTED)
    {
        Mission* mission = operation->mission();
        mission->clear();
        m_missionsService->saveMission(mission);
        operation->setState(MissionOperation::Succeeded);
    }
    else
    {
        qWarning() << "Vehicle" << vehicleId << " denied clear mission with ack" << ack.type;
        operation->setState(MissionOperation::Failed);
    }

    m_missionsService->endOperation(operation);
}

void MavlinkMissionClear::sendMissionClear(const QVariant& vehicleId)
{
    qDebug() << "sendMissionClear" << vehicleId;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
    if (!mavId)
        return;

    mavlink_message_t message;
    mavlink_mission_clear_all_t clearAll;

    clearAll.target_system = mavId;
    clearAll.target_component = MAV_COMP_ID_MISSIONPLANNER;

#ifdef MAVLINK_V2
    clearAll.mission_type = MAV_MISSION_TYPE_ALL;
#endif

    mavlink_msg_mission_clear_all_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                              &clearAll); // TODO: link channel
    emit sendMessage(message);
}

void MavlinkMissionClear::onOperationStarted(MissionOperation* operation)
{
    if (operation->type() != MissionOperation::Clear)
        return;

    QVariant vehicleId = operation->mission()->vehicleId();
    m_vehicleOperations[vehicleId] = operation;
    m_operationStates[operation] = WaitingAck;

    this->sendMissionClear(vehicleId);
}
