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

#ifndef LINK_CONFIGURATION_H
#define LINK_CONFIGURATION_H

#include <QScopedPointer>

#include "i_json_source.h"
#include "jerom_traits.h"
#include "link_factory.h"

namespace md::data_source
{
class LinkConfiguration
{
public:
    LinkConfiguration(const QString& fileName);

    LinkPtrMap createLinks();

    void checkHandlers();

private:
    QScopedPointer<IJsonSource> m_source;
    loodsman::LinkFactory m_linkFactory;
};

} // namespace md::data_source

#endif //LINK_CONFIGURATION_H
