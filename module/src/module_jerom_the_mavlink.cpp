#include "module_jerom_the_mavlink.h"

#include <QDebug>

#include "i_missions_service.h"
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

    missionService->registerMissionType(&domain::mavlink_mission::missionType);
    missionService->readAllMissions(); // TODO: IModule start() after init

    domain::MavlinkHandlerFactory factory(pTree, missionService);

    data_source::LinkConfiguration configuration(::linksFileName);
    m_transceiver = new domain::MavlinkTranscieverThreaded(
        new domain::MavlinkTransceiver(configuration.readLinks(), &factory, nullptr), this);
    m_transceiver->start();
}

void ModuleJeromTheMavlink::done()
{
    auto missionService = Locator::get<domain::IMissionsService>();
    Q_ASSERT(missionService);

    missionService->unregisterMissionType(&domain::mavlink_mission::missionType);

    m_transceiver->stop();
}
