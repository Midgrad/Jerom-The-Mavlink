#ifndef MAVLINK_HANDLER_FACTORY_H
#define MAVLINK_HANDLER_FACTORY_H

#include "i_mavlink_handlers_factory.h"

#include "i_property_tree.h"

namespace md::domain
{
class MavlinkHandlerFactory : public IMavlinkHandlerFactory
{
public:
    MavlinkHandlerFactory(IPropertyTree* pTree);

    QVector<IMavlinkHandler*> create(MavlinkHandlerContext* context) override;

private:
    IPropertyTree* const m_pTree;
};

} // namespace md::domain

#endif // MAVLINK_HANDLER_FACTORY_H
