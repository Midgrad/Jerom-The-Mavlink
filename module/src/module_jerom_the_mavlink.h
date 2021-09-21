#ifndef MODULE_JEROM_THE_MAVLINK_H
#define MODULE_JEROM_THE_MAVLINK_H

#include "i_mavlink_transceiver.h"
#include "i_module.h"

namespace md::app
{
class ModuleJeromTheMavlink
    : public QObject
    , public md::app::IModule
{
    Q_OBJECT
    Q_INTERFACES(md::app::IModule)
    Q_PLUGIN_METADATA(IID "Midgrad.ModuleJeromTheMavlink" FILE "meta.json")

public:
    Q_INVOKABLE ModuleJeromTheMavlink();

    void init() override;
    void done() override;

private:
    domain::IMavlinkTransceiver* m_transceiver = nullptr;
};
} // namespace md::app

#endif // MODULE_JEROM_THE_MAVLINK_H
