#ifndef LINK_CONFIGURATION_H
#define LINK_CONFIGURATION_H

#include <QScopedPointer>

#include "i_json_source.h"
#include "jerom_traits.h"

namespace md::data_source
{
class LinkConfiguration
{
public:
    LinkConfiguration(const QString& fileName);

    LinkPtrMap createLinks();

private:
    QScopedPointer<IJsonSource> m_source;
};

} // namespace md::data_source

#endif //LINK_CONFIGURATION_H
