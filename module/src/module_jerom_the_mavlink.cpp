#include "module_jerom_the_mavlink.h"

#include <QDebug>

#include "link_configuration.h"

#include "locator.h"

#include "mavlink_handlers_factory.h"
#include "mavlink_transceiver.h"
#include "mavlink_transceiver_threaded.h"

namespace
{
constexpr char linksFileName[] = "./link_config.json";
}

using namespace md::app;

ModuleJeromTheMavlink::ModuleJeromTheMavlink()
{
}

void ModuleJeromTheMavlink::init()
{
    auto pTree = Locator::get<md::domain::IPropertyTree>();
    domain::MavlinkHandlerFactory factory(pTree);

    data_source::LinkConfiguration configuration(::linksFileName);
    m_transceiver = new domain::MavlinkTranscieverThreaded(
        new domain::MavlinkTransceiver(configuration.readLinks(), &factory, nullptr), this);
    m_transceiver->start();
}

void ModuleJeromTheMavlink::done()
{
    m_transceiver->stop();
}
