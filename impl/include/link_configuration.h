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

    LinkPtrMap links();
    loodsman::LinkFactory* factory();

    void checkHandlers();
    void runHandlers();

private:
    LinkPtrMap createLinks();

    loodsman::LinkFactory m_factory;
    data_source::LinkPtrMap m_links;
    QScopedPointer<IJsonSource> m_source;
};

} // namespace md::data_source

#endif //LINK_CONFIGURATION_H
