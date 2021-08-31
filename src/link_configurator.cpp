#include "link_configurator.h"

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

void LinkConfigurator::read()
{
    QFile file(::path);
    file.open(QIODevice::ReadOnly | QIODevice::Text);

    m_document = QJsonDocument::fromJson(file.readAll());
    file.close();
}

QMap<QString, std::shared_ptr<loodsman::ILink>> LinkConfigurator::start()
{
    this->read();

    QMap<QString, std::shared_ptr<loodsman::ILink>> links;
    for (const QJsonValue& value : m_document.array())
    {
        QJsonObject linkConfig = value.toObject();

        loodsman::UdpLinkFactory factory(linkConfig.value(::port).toInt());
        std::shared_ptr<loodsman::ILink> link(factory.create());

        if (link)
            links[linkConfig.value(::name).toString()] = link;
    }

    return links;
}
