#ifndef MISSION_HANDLER_H
#define MISSION_HANDLER_H

#include "i_mavlink_handler.h"

namespace md::domain
{
class MissionHandler : public IMavlinkHandler
{
    Q_OBJECT

public:
    MissionHandler(MavlinkHandlerContext* context, QObject* parent = nullptr);
    ~MissionHandler() override;

    bool canParse(quint32 msgId) override;
    void parseMessage(const mavlink_message_t& message) override;

    void requestMissionCount(const QString& node);
    void sendMissionSetCurrent(const QString& node, int waypoint);

    void processMissionCurrent(const mavlink_message_t& message);
    void processMissionCount(const mavlink_message_t& message);

private:
    QStringList m_obtainedNodes;
};
} // namespace md::domain

#endif // MISSION_HANDLER_H
