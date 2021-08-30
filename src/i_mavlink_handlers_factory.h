#ifndef I_MAVLINK_HANDLER_FACTORY_H
#define I_MAVLINK_HANDLER_FACTORY_H

#include "i_mavlink_handler.h"

#include <QVector>

namespace jerom_mavlink::domain
{
class IMavlinkHandlerFactory
{
public:
    virtual ~IMavlinkHandlerFactory() = default;

    virtual QVector<IMavlinkHandler*> create(MavlinkHandlerContext* context) = 0;
};

} // namespace jerom_mavlink::domain

#endif // I_MAVLINK_HANDLER_FACTORY_H
