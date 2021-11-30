#ifndef MAVLINK_MISSION_COMMON_H
#define MAVLINK_MISSION_COMMON_H

#include "i_mavlink_handler.h"
#include "i_missions_service.h"

namespace md::domain
{
class MavlinkMissionCommon : public IMavlinkHandler
{
    Q_OBJECT

public:
    MavlinkMissionCommon(MavlinkHandlerContext* context, IMissionsService* missionsService,
                         QObject* parent = nullptr);

    void parse(const mavlink_message_t& message) final;

public slots:
    void onVehicleObtained(Vehicle* vehicle);

protected:
    void processMissionCurrent(const mavlink_message_t& message, const QVariant& vehicleId);
    void processMissionReached(const mavlink_message_t& message, const QVariant& vehicleId);

    void sendMissionSetCurrent(const QVariant& vehicleId, int index);

private slots:
    void onMissionAdded(Mission* mission);
    void onMissionRemoved(Mission* mission);

private:
    IMissionsService* const m_missionsService;
    QMap<QVariant, Mission*> m_vehicleMissions;
};
} // namespace md::domain

#endif // MAVLINK_MISSION_COMMON_H
