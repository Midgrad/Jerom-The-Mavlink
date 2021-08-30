#ifndef TELEMETRY_HANDLER_H
#define TELEMETRY_HANDLER_H

#include "i_mavlink_handler.h"

namespace jerom_mavlink::domain
{
class TelemetryHandler : public IMavlinkHandler
{
    Q_OBJECT

public:
    TelemetryHandler(MavlinkHandlerContext* context, QObject* parent = nullptr);
    ~TelemetryHandler() override;

    bool canParse(quint32 msgId) override;
    void parseMessage(const mavlink_message_t& message) override;

    void processAttitude(const mavlink_message_t& message);
    void processAltitude(const mavlink_message_t& message);
    void processGlobalPosition(const mavlink_message_t& message);
    void processSysStatus(const mavlink_message_t& message);
    void processVfrHud(const mavlink_message_t& message);
    void processGpsRaw(const mavlink_message_t& message);
    void processNavControllerOutput(const mavlink_message_t& message);
    void processMissionCurrent(const mavlink_message_t& message);
    void processMissionCount(const mavlink_message_t& message);
    void processHomePosition(const mavlink_message_t& message);

private:
    // TODO: this params should belong to a vehicle, not to the handler!
    bool m_hasAltitudeMessage;
};
} // namespace jerom_mavlink::domain

#endif // TELEMETRY_HANDLER_H
