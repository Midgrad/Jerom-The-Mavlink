#ifndef MAVLINK_HANDLER_FACTORY_H
#define MAVLINK_HANDLER_FACTORY_H

#include "i_mavlink_handlers_factory.h"
#include "i_missions_service.h"
#include "i_property_tree.h"

namespace md::domain
{
class MavlinkHandlerFactory : public IMavlinkHandlerFactory
{
public:
    MavlinkHandlerFactory(IPropertyTree* pTree, IMissionsService* missionsService);

    QVector<IMavlinkHandler*> create(MavlinkHandlerContext* context) override;

private:
    IPropertyTree* const m_pTree;
    IMissionsService* const m_missionsService;
};

} // namespace md::domain

#endif // MAVLINK_HANDLER_FACTORY_H
