#include "module_jerom_the_mavlink.h"

#include <QDebug>

#include "link_configurator.h"

#include "locator.h"

#include "mavlink_handlers_factory.h"
#include "mavlink_transceiver.h"
#include "mavlink_transceiver_threaded.h"

using namespace md::app;

ModuleJeromTheMavlink::ModuleJeromTheMavlink()
{
}

void ModuleJeromTheMavlink::init()
{
    auto links = md::domain::LinkConfigurator::start();

    auto pTree = Locator::get<md::domain::IPropertyTree>();
    domain::MavlinkHandlerFactory factory(pTree);

    m_transceiver = new domain::MavlinkTranscieverThreaded(new domain::MavlinkTransceiver(links,
                                                                                          &factory,
                                                                                          nullptr),
                                                           this);
    m_transceiver->start();
}

void ModuleJeromTheMavlink::done()
{
    m_transceiver->stop();
}
