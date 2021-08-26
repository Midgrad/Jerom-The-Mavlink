#ifndef I_MAVLINK_HANDLER_H
#define I_MAVLINK_HANDLER_H

#include <QObject>

#include <common/mavlink.h>

namespace jerom_mavlink::domain
{
class IMavlinkHandler : public QObject
{
    Q_OBJECT

public:
    IMavlinkHandler(QObject* parent) : QObject(parent)
    {
    }
    virtual ~IMavlinkHandler() = default;

    virtual bool canParse(quint32 msgId) = 0;
    virtual void parseMessage(const mavlink_message_t& message) = 0;
};

} // namespace jerom_mavlink::domain

#endif // I_MAVLINK_HANDLER_H
