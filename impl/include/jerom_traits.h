#ifndef JEROM_TRAITS_H
#define JEROM_TRAITS_H

#include "i_link_async.h"

#include <QMap>
#include <QSharedPointer>

namespace md::data_source
{
using LinkPtr = QSharedPointer<loodsman::ILinkAsync>;
using LinkPtrMap = QMap<QString, LinkPtr>;
} // namespace md::data_source

#endif // JEROM_TRAITS_H
