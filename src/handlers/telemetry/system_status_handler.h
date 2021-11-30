#ifndef SYSTEM_STATUS_HANDLER_H
#define SYSTEM_STATUS_HANDLER_H

#include "i_mavlink_handler.h"

namespace md::domain
{
class SystemStatusHandler : public IMavlinkHandler
{
    Q_OBJECT

public:
    SystemStatusHandler(MavlinkHandlerContext* context, QObject* parent = nullptr);
    ~SystemStatusHandler() override;

    void parse(const mavlink_message_t& message) override;

    void processSystemStatus(const mavlink_message_t& message);
};
} // namespace md::domain

#endif // SYSTEM_STATUS_HANDLER_H
