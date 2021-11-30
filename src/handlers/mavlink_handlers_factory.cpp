#include "mavlink_handlers_factory.h"

#include "heartbeat_handler.h"
#include "system_status_handler.h"
#include "telemetry_handler.h"

#include "mavlink_mission_clear.h"
#include "mavlink_mission_common.h"
#include "mavlink_mission_download.h"
#include "mavlink_mission_upload.h"

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
    auto mission = new MavlinkMissionCommon(context, m_missionsService);

    // Load mission for new MAVLINK vehicles
    QObject::connect(heartbeat, &HeartbeatHandler::vehicleObtained, mission,
                     &MavlinkMissionCommon::onVehicleObtained);

    QVector<IMavlinkHandler*> handlers;
    handlers.append(heartbeat);
    handlers.append(new TelemetryHandler(context));
    handlers.append(new SystemStatusHandler(context));

    handlers.append(mission);
    handlers.append(new MavlinkMissionUpload(context, m_missionsService));
    handlers.append(new MavlinkMissionDownload(context, m_missionsService));
    handlers.append(new MavlinkMissionClear(context, m_missionsService));

    return handlers;
}
