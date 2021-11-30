#include "mavlink_handlers_factory.h"

#include "heartbeat_handler.h"
#include "mission_handler.h"
#include "system_status_handler.h"
#include "telemetry_handler.h"

using namespace md::domain;

MavlinkHandlerFactory::MavlinkHandlerFactory(IPropertyTree* pTree,
                                             IMissionsService* missionsService,
                                             IVehiclesService* vehiclesService,
                                             ICommandsService* commandsService) :
    m_pTree(pTree),
    m_missionsService(missionsService),
    m_vehiclesService(vehiclesService),
    m_commandsService(commandsService)
{
}

QVector<IMavlinkHandler*> MavlinkHandlerFactory::create(MavlinkHandlerContext* context)
{
    context->pTree = m_pTree;

    auto heartbeat = new HeartbeatHandler(context, m_vehiclesService, m_commandsService);
    auto mission = new MissionHandler(context, m_missionsService);
    // Load mission for new MAVLINK vehicles
    QObject::connect(heartbeat, &HeartbeatHandler::vehicleObtained, mission,
                     &MissionHandler::onVehicleObtained);

    QVector<IMavlinkHandler*> handlers;
    handlers.append(heartbeat);
    handlers.append(mission);
    handlers.append(new TelemetryHandler(context));
    handlers.append(new SystemStatusHandler(context));
    return handlers;
}
