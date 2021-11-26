#include "module_jerom_the_mavlink.h"

#include <QDebug>

#include "locator.h"

#include "link_configuration.h"
#include "mavlink_handlers_factory.h"
#include "mavlink_mission_traits.h"
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
    auto pTree = Locator::get<domain::IPropertyTree>();
    Q_ASSERT(pTree);

    auto missionService = Locator::get<domain::IMissionsService>();
    Q_ASSERT(missionService);

    auto vehiclesService = Locator::get<domain::IVehiclesService>();
    Q_ASSERT(vehiclesService);

    auto commandsService = Locator::get<domain::ICommandsService>();
    Q_ASSERT(commandsService);

    missionService->registerMissionType(&domain::mission::mavlinkMissionType);

    domain::MavlinkHandlerFactory factory(pTree, missionService, vehiclesService, commandsService);

    auto configuration = new data_source::LinkConfiguration(::linksFileName);
    m_transceiver =
        new domain::MavlinkTranscieverThreaded(new domain::MavlinkTransceiver(configuration,
                                                                              &factory, nullptr),
                                               this);
}

void ModuleJeromTheMavlink::start()
{
    m_transceiver->start();
}

void ModuleJeromTheMavlink::done()
{
    auto missionService = Locator::get<domain::IMissionsService>();
    Q_ASSERT(missionService);

    missionService->unregisterMissionType(&domain::mission::mavlinkMissionType);

    m_transceiver->stop();

    delete m_transceiver;
}
