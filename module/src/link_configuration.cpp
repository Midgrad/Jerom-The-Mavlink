#include "link_configuration.h"

#include <QJsonArray>
#include <QJsonObject>

#include "json_source_file.h"
#include "udp_link_factory.h"

using namespace md::data_source;

namespace
{
constexpr char type[] = "type";
constexpr char name[] = "name";
constexpr char port[] = "port";

} // namespace

LinkConfiguration::LinkConfiguration(const QString& fileName) :
    m_source(new JsonSourceFile(fileName))
{
}

LinkPtrMap LinkConfiguration::readLinks()
{
    QJsonDocument document = m_source->read();

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
