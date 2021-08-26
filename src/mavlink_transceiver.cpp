#include "mavlink_transceiver.h"

#include <QJsonObject>

#include "link_factory.h"

using namespace jerom_mavlink::domain;

namespace
{
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

MavlinkTransciever::MavlinkTransciever(const QJsonArray& config, QObject* parent) :
    IMavlinkTransciever(parent)
{
    for (const QJsonValue& value : config)
    {
        QJsonObject linkConfig = value.toObject();

        loodsman::link_ptr link;
        loodsman::factory(link, ::linkTypeFromString(linkConfig.value(::type).toString()),
                          linkConfig.value(::port).toInt());
        if (link)
            m_links[linkConfig.value(::name).toString()] = link;
    }
}

void MavlinkTransciever::start()
{
    while (true)
    {
        std::string received_data(m_link->receive());
        emit result(QByteArray::fromStdString(received_data));
        //        qDebug() << "Data received";
    }
}

void MavlinkTransciever::stop()
{
}

void MavlinkTransciever::send(const QString& path, const QJsonObject& properties)
{
}
