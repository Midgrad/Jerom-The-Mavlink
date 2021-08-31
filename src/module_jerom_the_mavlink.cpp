#include "module_jerom_the_mavlink.h"

#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>

#include "udp_link_factory.h"

#include "locator.h"

#include "mavlink_handlers_factory.h"
#include "mavlink_transceiver.h"
#include "mavlink_transceiver_threaded.h"

namespace
{
constexpr char path[] = "./link_config.json";

constexpr char type[] = "type";
constexpr char name[] = "name";
constexpr char port[] = "port";

//loodsman::link_type linkTypeFromString(const QString& type)
//{
//    if (type == "udp")
//        return loodsman::link_type::udp;
//
//    // TODO: others
//
//    return loodsman::link_type::unknown;
//}
} // namespace

using namespace md::app;

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

    QMap<QString, std::shared_ptr<loodsman::ILink>> links;
    for (const QJsonValue& value : doc.array())
    {
        QJsonObject linkConfig = value.toObject();

        loodsman::UdpLinkFactory factory(linkConfig.value(::port).toInt());
        std::shared_ptr<loodsman::ILink> link(factory.create());

        if (link)
            links[linkConfig.value(::name).toString()] = link;
    }

    auto pTree = Locator::get<md::domain::IPropertyTree>();
    domain::MavlinkHandlerFactory factory(pTree);

    m_transceiver = new domain::MavlinkTranscieverThreaded(new domain::MavlinkTransceiver(links,
                                                                                          &factory,
                                                                                          nullptr),
                                                           this);
    m_transceiver->start();
}

void ModuleJeromTheMavlink::done()
{
    m_transceiver->stop();
}
