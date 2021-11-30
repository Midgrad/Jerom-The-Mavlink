#ifndef MAVLINK_MISSION_CLEAR_H
#define MAVLINK_MISSION_CLEAR_H

#include "mavlink_mission_base.h"

namespace md::domain
{
class MavlinkMissionClear : public MavlinkMissionBase
{
    Q_OBJECT

public:
    MavlinkMissionClear(MavlinkHandlerContext* context, IMissionsService* missionsService,
                        QObject* parent = nullptr);

protected:
    void parse(const mavlink_message_t& message, const QVariant& vehicleId) override;

    void processMissionAck(const mavlink_message_t& message, const QVariant& vehicleId);

    void sendMissionClear(const QVariant& vehicleId);

protected slots:
    void onOperationStarted(MissionOperation* operation) override;
};
} // namespace md::domain

#endif // MAVLINK_MISSION_CLEAR_H
