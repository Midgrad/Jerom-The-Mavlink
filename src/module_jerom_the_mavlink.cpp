#include "module_jerom_the_mavlink.h"

#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>

#include "link_factory.h"

#include "heartbeat_handler.h"
#include "telemetry_handler.h"

#include "mavlink_transceiver.h"
#include "mavlink_transceiver_threaded.h"

namespace
{
constexpr char path[] = "./link_config.json";

constexpr char type[] = "type";
constexpr char name[] = "name";
constexpr char port[] = "port";

loodsman::link_type linkTypeFromString(const QString& type)
{
    if (type == "udp")
        return loodsman::link_type::udp;

    // TODO: others

    return loodsman::link_type::unknown;
}
} // namespace

using namespace jerom_mavlink::app;

ModuleJeromTheMavlink::ModuleJeromTheMavlink()
{
}

void ModuleJeromTheMavlink::init()
{
    // TODO: to comman json config reader
    QFile file(::path);
    file.open(QIODevice::ReadOnly | QIODevice::Text);

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    file.close();

    QMap<QString, loodsman::LinkPtr> links;
    for (const QJsonValue& value : doc.array())
    {
        QJsonObject linkConfig = value.toObject();

        loodsman::LinkPtr link;
        loodsman::factory(link, ::linkTypeFromString(linkConfig.value(::type).toString()),
                          linkConfig.value(::port).toInt());
        if (link)
            links[linkConfig.value(::name).toString()] = link;
    }

    // TODO: to handlers factory
    QVector<domain::IMavlinkHandler*> handlers;

    handlers.append(new domain::HeartbeatHandler());
    handlers.append(new domain::TelemetryHandler());

    // TODO: wrap transceiver with threaed decorator
    m_transciever = new domain::MavlinkTranscieverThreaded(new domain::MavlinkTransciever(links,
                                                                                          handlers,
                                                                                          nullptr),
                                                           this);
    m_transciever->start();
}

void ModuleJeromTheMavlink::done()
{
    m_transciever->stop();
}
