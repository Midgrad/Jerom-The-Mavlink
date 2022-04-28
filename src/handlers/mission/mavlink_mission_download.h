#ifndef MAVLINK_MISSION_DOWNLOAD_H
#define MAVLINK_MISSION_DOWNLOAD_H

#include "mavlink_item_convertors.h"
#include "mavlink_mission_base.h"
#include "mission_operation.h"

namespace md::domain
{
class MavlinkMissionDownload : public MavlinkMissionBase
{
    Q_OBJECT

public:
    MavlinkMissionDownload(MavlinkHandlerContext* context, IMissionsService* missionsService,
                           QObject* parent = nullptr);

protected:
    void parse(const mavlink_message_t& message, const QVariant& vehicleId) override;

    void processMissionCount(const mavlink_message_t& message, const QVariant& vehicleId);
    void processMissionItem(const mavlink_message_t& message, const QVariant& vehicleId);

    void sendMissionRequest(const QVariant& vehicleId);
    void sendMissionItemRequest(const QVariant& vehicleId, int index);

protected slots:
    void onOperationStarted(MissionOperation* operation) override;

private:
    MavlinkItemConvertorsPool m_convertorsPool;
};
} // namespace md::domain

#endif // MAVLINK_MISSION_DOWNLOAD_H
