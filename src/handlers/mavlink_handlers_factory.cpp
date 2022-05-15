#include "mavlink_handlers_factory.h"

#include "heartbeat_handler.h"
#include "system_status_handler.h"
#include "telemetry_handler.h"

#include "mavlink_mission_clear.h"
#include "mavlink_mission_common.h"
#include "mavlink_mission_download.h"
#include "mavlink_mission_upload.h"

using namespace md::domain;

MavlinkHandlerFactory::MavlinkHandlerFactory(IPropertyTree* pTree, IMissionsService* missions,
                                             IVehiclesService* vehicles,
                                             IVehicleMissions* vehicleMissions,
                                             ICommandsService* commands) :
    m_pTree(pTree),
    m_missions(missions),
    m_vehicles(vehicles),
    m_vehicleMissions(vehicleMissions),
    m_commands(commands)
{
}

QVector<IMavlinkHandler*> MavlinkHandlerFactory::create(MavlinkHandlerContext* context)
{
    context->pTree = m_pTree;

    auto heartbeat = new HeartbeatHandler(context, m_vehicles, m_commands);

    // Load mission for new MAVLINK vehicles
    QObject::connect(heartbeat, &HeartbeatHandler::vehicleObtained, m_vehicleMissions,
                     &IVehicleMissions::obtainVehicle);

    QVector<IMavlinkHandler*> handlers;
    handlers.append(heartbeat);
    handlers.append(new TelemetryHandler(context));
    handlers.append(new SystemStatusHandler(context));

    handlers.append(new MavlinkMissionCommon(context, m_vehicleMissions));
    handlers.append(new MavlinkMissionUpload(context, m_missions));
    handlers.append(new MavlinkMissionDownload(context, m_missions));
    handlers.append(new MavlinkMissionClear(context, m_missions));

    return handlers;
}
