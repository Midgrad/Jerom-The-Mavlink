#ifndef MODULE_JEROM_THE_MAVLINK_H
#define MODULE_JEROM_THE_MAVLINK_H

#include "i_communication_protocol.h"
#include "i_module.h"

#include "mavlink_mission_pattern_factory.h"

namespace md::app
{
class ModuleJeromTheMavlink
    : public QObject
    , public IModule
{
    Q_OBJECT
    Q_INTERFACES(md::app::IModule)
    Q_PLUGIN_METADATA(IID "Midgrad.ModuleJeromTheMavlink" FILE "meta.json")

public:
    Q_INVOKABLE ModuleJeromTheMavlink();
    ~ModuleJeromTheMavlink();

    void init() override;
    void start() override;
    void done() override;

private:
    data_source::ICommunicationProtocol* m_protocol = nullptr;
    domain::MavlinkMissionPatternFactory m_patternFactory;
};
} // namespace md::app

#endif // MODULE_JEROM_THE_MAVLINK_H
