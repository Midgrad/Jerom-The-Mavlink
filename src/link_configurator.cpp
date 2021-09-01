#include "link_configurator.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMap>

#include "udp_link_factory.h"

using namespace md::domain;

// TODO: to common json config reader - https://github.com/Midgrad/kjarni/issues/2

namespace
{
constexpr char path[] = "./link_config.json";

constexpr char type[] = "type";
constexpr char name[] = "name";
constexpr char port[] = "port";

} // namespace

QJsonDocument LinkConfigurator::read()
{
    QFile file(::path);
    file.open(QIODevice::ReadOnly | QIODevice::Text);

    QJsonDocument document = QJsonDocument::fromJson(file.readAll());
    file.close();

    return document;
}

QMap<QString, std::shared_ptr<loodsman::ILink>> LinkConfigurator::start()
{
    QJsonDocument document = LinkConfigurator::read();

    QMap<QString, std::shared_ptr<loodsman::ILink>> links;
    for (const QJsonValue& value : document.array())
    {
        QJsonObject linkConfig = value.toObject();

        loodsman::UdpLinkFactory factory(linkConfig.value(::port).toInt());
        std::shared_ptr<loodsman::ILink> link(factory.create());

        if (link)
            links[linkConfig.value(::name).toString()] = link;
    }

    return links;
}
