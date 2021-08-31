#ifndef HEARTBEAT_HANDLER_H
#define HEARTBEAT_HANDLER_H

#include "i_mavlink_handler.h"
#include "i_mode_helper.h"

#include <QMap>

namespace md::domain
{
class HeartbeatHandler : public IMavlinkHandler
{
    Q_OBJECT

public:
    HeartbeatHandler(MavlinkHandlerContext* context, QObject* parent = nullptr);
    ~HeartbeatHandler() override;

    bool canParse(quint32 msgId) override;
    void parseMessage(const mavlink_message_t& message) override;

    void sendMode(const QString& node, const QString& mode);

    void processHeartbeat(const mavlink_message_t& message);

private:
    QMap<quint8, QSharedPointer<data_source::IModeHelper>> m_modeHelpers;
    QMap<quint8, quint8> m_baseModes;
};
} // namespace md::domain

#endif // HEARTBEAT_HANDLER_H
