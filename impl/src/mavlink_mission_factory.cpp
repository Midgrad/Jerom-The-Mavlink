#include "mavlink_mission_factory.h"

using namespace md::domain;

MavlinkMissionFactory::MavlinkMissionFactory()
{
}

Mission* MavlinkMissionFactory::createMission()
{
    auto mission = new Mission(mavlinkMissionType, "New mission");

    this->createRouteForMission(mission);

    return mission;
}

Route* MavlinkMissionFactory::createRouteForMission(Mission* mission)
{
    auto route = new Route(mission->name() + " " + QObject::tr("route"));
    mission->setRoute(route);
    return route;
}

Waypoint* MavlinkMissionFactory::createWaypointForRoute(Route* route)
{
    auto waypoint = new Waypoint(QObject::tr("WPT %1").arg(route->count()));
    route->addWaypoint(waypoint);
    return waypoint;
}
