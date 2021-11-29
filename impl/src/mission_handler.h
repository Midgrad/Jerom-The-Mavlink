#ifndef MISSION_HANDLER_H
#define MISSION_HANDLER_H

#include "i_mavlink_handler.h"
#include "i_missions_service.h"
#include "i_vehicles_repository.h"
#include "mavlink_item_convertors.h"

namespace md::domain
{
// FIXME: MissonRouteHandler
class MissionHandler : public IMavlinkHandler
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

    MissionHandler(MavlinkHandlerContext* context, IMissionsService* missionsService,
                   QObject* parent = nullptr);
    ~MissionHandler() override;

    bool canParse(quint32 msgId) override;
    void parseMessage(const mavlink_message_t& message) override;

    void sendMissionRequest(const QVariant& vehicleId);
    void sendMissionItemRequest(const QVariant& vehicleId, int index);
    void sendAck(const QVariant& vehicleId, MAV_MISSION_RESULT type);
    void sendMissionSetCurrent(const QVariant& vehicleId, int index);
    void sendMissionCount(const QVariant& vehicleId, int count);
    void sendMissionItem(const QVariant& vehicleId, RouteItem* routeItem, int index);

    void processMissionAck(const mavlink_message_t& message);
    void processMissionRequest(const mavlink_message_t& message);
    void processMissionItem(const mavlink_message_t& message);
    void processMissionCurrent(const mavlink_message_t& message);
    void processMissionCount(const mavlink_message_t& message);
    void processMissionReached(const mavlink_message_t& message);

public slots:
    void onVehicleObtained(Vehicle* vehicle);

private slots:
    void onMissionAdded(Mission* mission);
    void onMissionRemoved(Mission* mission);

    void uploadItem(Mission* mission, int index);
    void upload(Mission* mission);
    void download(Mission* mission);
    void cancel(Mission* mission);

private:
    IMissionsService* const m_missionsService;
    MavlinkItemConvertorsPool m_convertors;
    QMap<QVariant, Mission*> m_vehicleMissions;
    QMap<Mission*, State> m_missionStates;
};
} // namespace md::domain

#endif // MISSION_HANDLER_H
