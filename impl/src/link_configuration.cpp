//     Copyright (c) Ilia Kuklev aka Elias Alberts 2021.
//     nnhw667@gmail.com
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "link_configuration.h"

#include <QJsonArray>
#include <QJsonObject>

#include "json_source_file.h"
#include "link_factory.h"

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

LinkPtrMap LinkConfiguration::createLinks()
{
    QJsonDocument document = m_source->read();

    QMap<QString, LinkPtr> links;
    for (const QJsonValue& value : document.array())
    {
        QJsonObject linkConfig = value.toObject();

        //        loodsman::LinkFactory factory;
        LinkPtr link(
            m_linkFactory.createIp(loodsman::LinkType::udp, linkConfig.value(::port).toInt()));

        if (link)
            links[linkConfig.value(::name).toString()] = link;
    }

    return links;
}

void LinkConfiguration::checkHandlers()
{
    m_linkFactory.checkHandlers();
}
