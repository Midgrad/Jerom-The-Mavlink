#ifndef MAVLINK_HANDLER_FACTORY_H
#define MAVLINK_HANDLER_FACTORY_H

#include "i_mavlink_handlers_factory.h"

#include "i_property_tree.h"

namespace jerom_mavlink::domain
{
class MavlinkHandlerFactory : public IMavlinkHandlerFactory
{
public:
    MavlinkHandlerFactory(kjarni::domain::IPropertyTree* pTree);

    QVector<IMavlinkHandler*> create() override;

private:
    kjarni::domain::IPropertyTree* const m_pTree;
};

} // namespace jerom_mavlink::domain

#endif // MAVLINK_HANDLER_FACTORY_H
