#ifndef LINK_CONFIGURATION_H
#define LINK_CONFIGURATION_H

#include "i_link.h"

#include <memory>

#include <QJsonDocument>
#include <QString>

namespace md::domain
{
using linkPtr = std::shared_ptr<loodsman::ILink>;

// TODO: make LinkConfiguration(const QJsonDocument& document); interface after
// common json config reader - https://github.com/Midgrad/kjarni/issues/2
class LinkConfiguration
{
public:
    LinkConfiguration() = default;

    static QJsonDocument read();

    static QMap<QString, linkPtr> start();

private:
};

} // namespace md::domain

#endif //LINK_CONFIGURATION_H
