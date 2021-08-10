#ifndef MODULE_JEROM_THE_MAVLINK_H
#define MODULE_JEROM_THE_MAVLINK_H

#include "i_module.h"

namespace jerom_mavlink::app
{
class ModuleJeromTheMavlink
    : public QObject
    , public kjarni::app::IModule
{
    Q_OBJECT
    Q_INTERFACES(kjarni::app::IModule)
    Q_PLUGIN_METADATA(IID "Midgrad.ModuleJeromTheMavlink" FILE "meta.json")

public:
    Q_INVOKABLE ModuleJeromTheMavlink();

    void init() override;
    void done() override;

    QJsonObject qmlEntries() const override;
};
} // namespace jerom_mavlink::app

#endif // MODULE_JEROM_THE_MAVLINK_H
