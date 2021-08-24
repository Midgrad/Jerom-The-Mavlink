#ifndef TELEMETRY_HANDLER_H
#define TELEMETRY_HANDLER_H

#include "i_mavlink_handler.h"
#include "i_property_tree.h"

namespace jerom_mavlink
{
class TelemetryHandler : public IMavlinkHandler
{
    Q_OBJECT

public:
    TelemetryHandler();
    ~TelemetryHandler() override;

    bool canParse(quint32 msgId) override;
    void parseMessage(const mavlink_message_t& message) override;

    void processAttitude(const mavlink_message_t& message);
    void processAltitude(const mavlink_message_t& message);
    void processGlobalPosition(const mavlink_message_t& message);

private:
    kjarni::domain::IPropertyTree* m_pTree;
};
} // namespace jerom_mavlink

#endif // TELEMETRY_HANDLER_H
