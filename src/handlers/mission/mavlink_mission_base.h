#ifndef MAVLINK_MISSION_BASE_H
#define MAVLINK_MISSION_BASE_H

#include "i_mavlink_handler.h"
#include "i_missions_service.h"

namespace md::domain
{
class MavlinkMissionBase : public IMavlinkHandler
{
    Q_OBJECT

public:
    enum State
    {
        Idle,
        WaitingCount,
        WaitingItem,
        WaitingRequest,
        WaitingAck
    };

    MavlinkMissionBase(MavlinkHandlerContext* context, IMissionsService* missionsService,
                       QObject* parent);

    void parse(const mavlink_message_t& message) final;

protected:
    virtual void parse(const mavlink_message_t& message, const QVariant& vehicleId);

    void sendAck(const QVariant& vehicleId, MAV_MISSION_RESULT type);

protected slots:
    virtual void onOperationStarted(MissionOperation* operation);
    virtual void onOperationEnded(MissionOperation* operation);

protected:
    IMissionsService* const m_missionsService;

    QMap<QVariant, MissionOperation*> m_vehicleOperations;
    QMap<MissionOperation*, State> m_operationStates;
};
} // namespace md::domain

#endif // MAVLINK_MISSION_BASE_H
