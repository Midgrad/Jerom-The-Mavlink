#include "link_configuration.h"

#include <QDebug>

#include <QJsonArray>
#include <QJsonObject>

#include "json_source_file.h"
#include "link_factory.h"

using namespace md::data_source;

namespace
{
constexpr char type[] = "type";
constexpr char name[] = "name";
constexpr char localPort[] = "local_port";

} // namespace

LinkConfiguration::LinkConfiguration(const QString& fileName) :
    m_source(new JsonSourceFile(fileName))
{
}

LinkPtrMap LinkConfiguration::createLinks()
{
    QJsonDocument document = m_source->read();

    QMap<QString, LinkPtr> links;
    for (const QJsonValue& value : document.array())
    {
        QJsonObject linkConfig = value.toObject();

        loodsman::LinkFactory factory;

        QString type = (linkConfig.value(::type).toString());
        LinkPtr link;

        if (type == "udp")
            link = LinkPtr(
                factory.create(loodsman::LinkType::udp, linkConfig.value(::localPort).toInt()));
        else if (type == "tcp")
            link = LinkPtr(
                factory.create(loodsman::LinkType::tcp, linkConfig.value(::localPort).toInt()));
        else
            qWarning() << "Wrong link type in " << linkConfig.value(::name).toString();

        if (link)
            links[linkConfig.value(::name).toString()] = link;
    }

    return links;
}
