#ifndef HEARTBEAT_HANDLER_H
#define HEARTBEAT_HANDLER_H

#include "i_mavlink_handler.h"
#include "i_property_tree.h"

namespace jerom_mavlink::domain
{
class HeartbeatHandler : public IMavlinkHandler
{
    Q_OBJECT

public:
    HeartbeatHandler();
    ~HeartbeatHandler() override;

    bool canParse(quint32 msgId) override;
    void parseMessage(const mavlink_message_t& message) override;

    void processHeartbeat(const mavlink_message_t& message);

private:
    kjarni::domain::IPropertyTree* m_pTree;
};
} // namespace jerom_mavlink::domain

#endif // HEARTBEAT_HANDLER_H
