#ifndef MISSION_HANDLER_H
#define MISSION_HANDLER_H

#include "i_mavlink_handler.h"
#include "i_missions_service.h"

namespace md::domain
{
class MissionHandler : public IMavlinkHandler
{
    Q_OBJECT

public:
    MissionHandler(MavlinkHandlerContext* context, IMissionsService* missionsService,
                   QObject* parent = nullptr);
    ~MissionHandler() override;

    bool canParse(quint32 msgId) override;
    void parseMessage(const mavlink_message_t& message) override;

    void sendMissionRequestList(const QString& node);
    void sendMissionItemRequest(const QString& node, int index);
    void sendMissionSetCurrent(const QString& node, int waypoint);

    void processMissionCurrent(const mavlink_message_t& message);
    void processMissionCount(const mavlink_message_t& message);
    void processMissionReached(const mavlink_message_t& message);

public slots:
    void subscribeMission(Mission* mission);
    void unsubscribeMission(Mission* mission);

private:
    QStringList m_obtainedNodes;
    QMap<QString, Mission*> m_downloadingMissions;
    QMap<QString, Mission*> m_uploadingMissions;
};
} // namespace md::domain

#endif // MISSION_HANDLER_H
