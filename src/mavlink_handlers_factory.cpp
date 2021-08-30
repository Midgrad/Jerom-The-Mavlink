#include "mavlink_handlers_factory.h"

#include "heartbeat_handler.h"
#include "telemetry_handler.h"

using namespace jerom_mavlink::domain;

MavlinkHandlerFactory::MavlinkHandlerFactory(kjarni::domain::IPropertyTree* pTree) : m_pTree(pTree)
{
}

QVector<IMavlinkHandler*> MavlinkHandlerFactory::create(MavlinkHandlerContext* context)
{
    context->pTree = m_pTree;

    QVector<IMavlinkHandler*> handlers;
    handlers.append(new HeartbeatHandler(context));
    handlers.append(new TelemetryHandler(context));
    return handlers;
}
