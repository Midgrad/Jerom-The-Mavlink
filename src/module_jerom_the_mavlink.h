#ifndef MODULE_JEROM_THE_MAVLINK_H
#define MODULE_JEROM_THE_MAVLINK_H

#include "i_mavlink_transceiver.h"
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

private:
    domain::IMavlinkTransciever* m_transciever = nullptr;
};
} // namespace jerom_mavlink::app

#endif // MODULE_JEROM_THE_MAVLINK_H
