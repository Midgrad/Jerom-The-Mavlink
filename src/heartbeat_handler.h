#ifndef HEARTBEAT_HANDLER_H
#define HEARTBEAT_HANDLER_H

#include "i_mavlink_handler.h"

namespace jerom_mavlink::domain
{
class HeartbeatHandler : public IMavlinkHandler
{
    Q_OBJECT

public:
    HeartbeatHandler(QObject* parent = nullptr);
    ~HeartbeatHandler() override;

    bool canParse(quint32 msgId) override;
    void parseMessage(const mavlink_message_t& message) override;

    void processHeartbeat(const mavlink_message_t& message);
};
} // namespace jerom_mavlink::domain

#endif // HEARTBEAT_HANDLER_H
