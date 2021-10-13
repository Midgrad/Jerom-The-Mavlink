#ifndef MAVLINK_HANDLER_FACTORY_H
#define MAVLINK_HANDLER_FACTORY_H

#include "i_command_service.h"
#include "i_mavlink_handlers_factory.h"
#include "i_missions_service.h"
#include "i_property_tree.h"
#include "i_vehicles_service.h"

namespace md::domain
{
class MavlinkHandlerFactory : public IMavlinkHandlerFactory
{
public:
    MavlinkHandlerFactory(IPropertyTree* pTree, IMissionsService* missionsService,
                          IVehiclesService* vehiclesService, ICommandsService* commandsService);

    QVector<IMavlinkHandler*> create(MavlinkHandlerContext* context) override;

private:
    IPropertyTree* const m_pTree;
    IMissionsService* const m_missionsService;
    IVehiclesService* const m_vehiclesService;
    ICommandsService* const m_commandsService;
};

} // namespace md::domain

#endif // MAVLINK_HANDLER_FACTORY_H
