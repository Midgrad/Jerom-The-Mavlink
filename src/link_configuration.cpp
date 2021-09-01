#include "link_configuration.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMap>

#include "udp_link_factory.h"

using namespace md::domain;

namespace
{
constexpr char path[] = "./link_config.json";

constexpr char type[] = "type";
constexpr char name[] = "name";
constexpr char port[] = "port";

} // namespace

QJsonDocument LinkConfiguration::read()
{
    QFile file(::path);
    file.open(QIODevice::ReadOnly | QIODevice::Text);

    QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    file.close();

    return document;
}

QMap<QString, LinkPtr> LinkConfiguration::start()
{
    QJsonDocument document = LinkConfiguration::read();

    QMap<QString, LinkPtr> links;
    for (const QJsonValue& value : document.array())
    {
        QJsonObject linkConfig = value.toObject();

        loodsman::UdpLinkFactory factory(linkConfig.value(::port).toInt());
        LinkPtr link(factory.create());

        if (link)
            links[linkConfig.value(::name).toString()] = link;
    }

    return links;
}
