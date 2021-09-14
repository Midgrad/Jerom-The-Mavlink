#include "mavlink_mission_factory.h"

using namespace md::domain;

MavlinkMissionFactory::MavlinkMissionFactory()
{
}

Mission* MavlinkMissionFactory::create()
{
    auto mission = new Mission(mavlinkType, "New mission");
    auto route = new Route(mission->name() + " " + QObject::tr("route"));

    mission->setRoute(route);

    return mission;
}
