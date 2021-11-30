#include "mavlink_mission_base.h"

#include <QDebug>

using namespace md::domain;

MavlinkMissionBase::MavlinkMissionBase(MavlinkHandlerContext* context,
                                       IMissionsService* missionsService, QObject* parent) :
    IMavlinkHandler(context, parent),
    m_missionsService(missionsService)
{
    Q_ASSERT(missionsService);

    connect(m_missionsService, &IMissionsService::operationStarted, this,
            &MavlinkMissionBase::onOperationStarted);
    connect(m_missionsService, &IMissionsService::operationStarted, this,
            &MavlinkMissionBase::onOperationEnded);
}

void MavlinkMissionBase::parse(const mavlink_message_t& message)
{
    QString vehicleId = m_context->vehicleIds.value(message.sysid).toString();
    if (vehicleId.isNull())
        return;

    return this->parse(message, vehicleId); // TODO: MavlinkVehicleMessage
}

void MavlinkMissionBase::parse(const mavlink_message_t& message, const QVariant& vehicleId)
{
    Q_UNUSED(message);
    Q_UNUSED(vehicleId);
}

void MavlinkMissionBase::sendAck(const QVariant& vehicleId, MAV_MISSION_RESULT type)
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

void MavlinkMissionBase::onOperationStarted(MissionOperation* operation)
{
    Q_UNUSED(operation)
}

void MavlinkMissionBase::onOperationEnded(MissionOperation* operation)
{
    if (!m_operationStates.contains(operation))
        return;

    m_operationStates.remove(operation);
    m_vehicleOperations.remove(m_vehicleOperations.key(operation));
}
