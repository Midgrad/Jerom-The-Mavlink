#include "mavlink_handlers_factory.h"

#include "heartbeat_handler.h"
#include "telemetry_handler.h"

using namespace jerom_mavlink::domain;

MavlinkHandlerFactory::MavlinkHandlerFactory(kjarni::domain::IPropertyTree* pTree) : m_pTree(pTree)
{
}

QVector<IMavlinkHandler*> MavlinkHandlerFactory::create()
{
    QVector<IMavlinkHandler*> handlers;

    handlers.append(new HeartbeatHandler());
    handlers.append(new TelemetryHandler());

    for (IMavlinkHandler* handler : handlers)
    {
        QObject::connect(handler, &IMavlinkHandler::propertiesObtained, m_pTree,
                         &kjarni::domain::IPropertyTree::appendProperties);
    }

    return handlers;
}
