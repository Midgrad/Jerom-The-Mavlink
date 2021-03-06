#ifndef I_MAVLINK_HANDLER_H
#define I_MAVLINK_HANDLER_H

#include <common/mavlink.h>

#include <QObject>

#include "i_property_tree.h"
#include "vehicle.h"

namespace md::domain // FIXME: data_source
{
struct MavlinkHandlerContext
{
    quint8 systemId = 255;
    quint8 compId = 0;

    IPropertyTree* pTree = nullptr;
    QMap<quint8, QVariant> vehicleIds;
};

class IMavlinkHandler : public QObject // TODO: IMavlinkMessage
{
    Q_OBJECT

public:
    IMavlinkHandler(MavlinkHandlerContext* context, QObject* parent) :
        QObject(parent),
        m_context(context)
    {
    }
    virtual ~IMavlinkHandler() = default;

    virtual void parse(const mavlink_message_t& message) = 0;

signals:
    void sendMessage(mavlink_message_t message);

protected:
    MavlinkHandlerContext* const m_context;
};

} // namespace md::domain

#endif // I_MAVLINK_HANDLER_H
