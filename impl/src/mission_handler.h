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
    MissionHandler(MavlinkHandlerContext* context, IMissionsService* missionsService,
                   IVehiclesService* vehiclesService, QObject* parent = nullptr);
    ~MissionHandler() override;

    bool canParse(quint32 msgId) override;
    void parseMessage(const mavlink_message_t& message) override;

    void sendMissionRequest(const QString& node);
    void sendMissionItemRequest(const QString& node, int index);
    void sendAck(const QString& node, MAV_MISSION_RESULT type);
    void sendMissionSetCurrent(const QString& node, int waypoint);

    void processMissionItem(const mavlink_message_t& message);
    void processMissionCurrent(const mavlink_message_t& message);
    void processMissionCount(const mavlink_message_t& message);
    void processMissionReached(const mavlink_message_t& message);

signals:
    void statusUpdate(QVariant missionId, MissionStatus status);

private slots:
    void upload(Mission* mission);
    void download(Mission* mission);
    void cancel(Mission* mission);

private:
    IVehiclesService* const m_vehiclesService;
    MavlinkItemConvertorsPool m_convertors;
    QStringList m_obtainedNodes;
    QMap<QString, Mission*> m_downloadingMissions;
    QMap<QString, Mission*> m_uploadingMissions;
    QMap<QString, MissionStatus> m_statuses;
};
} // namespace md::domain

#endif // MISSION_HANDLER_H
