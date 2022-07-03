#ifndef MAVLINK_MISSION_COMMON_H
#define MAVLINK_MISSION_COMMON_H

#include "i_mavlink_handler.h"
#include "i_vehicle_missions.h"

namespace md::domain
{
class MavlinkMissionCommon : public IMavlinkHandler
{
    Q_OBJECT

public:
    MavlinkMissionCommon(MavlinkHandlerContext* context, IVehicleMissions* vehicleMissions,
                         QObject* parent = nullptr);

    void parse(const mavlink_message_t& message) final;

protected:
    void processMissionCurrent(const mavlink_message_t& message, const QVariant& vehicleId);
    void processMissionReached(const mavlink_message_t& message, const QVariant& vehicleId);
    void processHomePosition(const mavlink_message_t& message, const QVariant& vehicleId);
    void processTargetPosition(const mavlink_message_t& message, const QVariant& vehicleId);

    void sendMissionSetCurrent(const QVariant& vehicleId, int index);
    void sendNavTo(const QVariant& vehicleId, double latitude, double longitude, float altitude);

private slots:
    void onMissionAdded(Mission* mission);
    void onMissionRemoved(Mission* mission);

private:
    IVehicleMissions* const m_vehicleMissions;
};
} // namespace md::domain

#endif // MAVLINK_MISSION_COMMON_H
