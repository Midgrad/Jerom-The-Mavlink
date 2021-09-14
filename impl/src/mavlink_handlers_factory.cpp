#include "mavlink_handlers_factory.h"

#include "heartbeat_handler.h"
#include "mission_handler.h"
#include "system_status_handler.h"
#include "telemetry_handler.h"

using namespace md::domain;

MavlinkHandlerFactory::MavlinkHandlerFactory(IPropertyTree* pTree,
                                             IMissionsService* missionsService) :
    m_pTree(pTree),
    m_missionsService(missionsService)
{
}

QVector<IMavlinkHandler*> MavlinkHandlerFactory::create(MavlinkHandlerContext* context)
{
    context->pTree = m_pTree;

    QVector<IMavlinkHandler*> handlers;
    handlers.append(new HeartbeatHandler(context));
    handlers.append(new TelemetryHandler(context));
    handlers.append(new MissionHandler(context, m_missionsService));
    handlers.append(new SystemStatusHandler(context));
    return handlers;
}
