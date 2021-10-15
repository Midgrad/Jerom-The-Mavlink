#ifndef MISSION_HANDLER_H
#define MISSION_HANDLER_H

#include "i_mavlink_handler.h"
#include "i_missions_service.h"
#include "i_vehicles_service.h"
#include "mavlink_item_convertors.h"

namespace md::domain
{
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

    void sendMissionRequest(const QString& vehicleId);
    void sendMissionItemRequest(const QString& vehicleId, int index);
    void sendAck(const QString& vehicleId, MAV_MISSION_RESULT type);
    void sendMissionSetCurrent(const QString& vehicleId, int waypoint);

    void processMissionItem(const mavlink_message_t& message);
    void processMissionCurrent(const mavlink_message_t& message);
    void processMissionCount(const mavlink_message_t& message);
    void processMissionReached(const mavlink_message_t& message);

public slots:
    void onVehicleObtained(Vehicle* vehicle);

private slots:
    void onMissionAdded(Mission* mission);
    void onMissionRemoved(Mission* mission);

    void upload(Mission* mission);
    void download(Mission* mission);
    void cancel(Mission* mission);

private:
    IMissionsService* const m_missionsService;
    MavlinkItemConvertorsPool m_convertors;
    QMap<QString, Mission*> m_vehicleMissions;
    QMap<Mission*, State> m_missionStates;
};
} // namespace md::domain

#endif // MISSION_HANDLER_H
