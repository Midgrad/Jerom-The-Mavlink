#ifndef TELEMETRY_HANDLER_H
#define TELEMETRY_HANDLER_H

#include "i_mavlink_handler.h"

namespace md::domain
{
class TelemetryHandler : public IMavlinkHandler
{
    Q_OBJECT

public:
    TelemetryHandler(MavlinkHandlerContext* context, QObject* parent = nullptr);
    ~TelemetryHandler() override;

    void parse(const mavlink_message_t& message) override;

    void processAttitude(const mavlink_message_t& message);
    void processAltitude(const mavlink_message_t& message);
    void processGlobalPosition(const mavlink_message_t& message);
    void processVfrHud(const mavlink_message_t& message);
    void processGpsRaw(const mavlink_message_t& message);
    void processNavControllerOutput(const mavlink_message_t& message);

private:
    // TODO: this params should belong to a vehicle, not to the handler!
    bool m_hasAltitudeMessage;
};
} // namespace md::domain

#endif // TELEMETRY_HANDLER_H
