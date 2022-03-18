#ifndef MAVLINK_MISSION_UPLOAD_H
#define MAVLINK_MISSION_UPLOAD_H

#include "mavlink_item_convertors.h"
#include "mavlink_mission_base.h"
#include "mission_operation.h"

namespace md::domain
{
class MavlinkMissionUpload : public MavlinkMissionBase
{
    Q_OBJECT

public:
    MavlinkMissionUpload(MavlinkHandlerContext* context, IMissionsService* missionsService,
                         QObject* parent = nullptr);

protected:
    void parse(const mavlink_message_t& message, const QVariant& vehicleId) override;

    void processMissionRequest(const mavlink_message_t& message, const QVariant& vehicleId);
    void processMissionAck(const mavlink_message_t& message, const QVariant& vehicleId);

    void sendMissionCount(const QVariant& vehicleId, int count);
    void sendMissionItem(const QVariant& vehicleId, MissionRouteItem* routeItem, int index);

protected slots:
    void onOperationStarted(MissionOperation* operation) override;

private:
    MavlinkItemConvertorsPool m_convertors;
};
} // namespace md::domain

#endif // MAVLINK_MISSION_UPLOAD_H
