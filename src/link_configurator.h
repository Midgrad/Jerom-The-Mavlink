#ifndef LINK_CONFIG_PARSER_H
#define LINK_CONFIG_PARSER_H

#include "i_link.h"

#include <memory>

#include <QJsonDocument>
#include <QString>

namespace md::domain
{
class LinkConfigurator
{
public:
    LinkConfigurator() = default;

    void read();

    QMap<QString, std::shared_ptr<loodsman::ILink>> start();

private:
    QJsonDocument m_document;
};

} // namespace md::domain

#endif //LINK_CONFIG_PARSER_H
