#include "mavlink_mission_factory.h"

#include "utils.h"

using namespace md::domain;

MavlinkMissionFactory::MavlinkMissionFactory()
{
}

Mission* MavlinkMissionFactory::createMission(const QString& name)
{
    auto mission = new Mission(mavlink_mission::missionType, utils::nameToId(name), name);

    this->createRouteForMission(mission);
    this->createWaypointForRoute(mission->route(), &mavlink_mission::waypoint);

    return mission;
}

Route* MavlinkMissionFactory::createRouteForMission(Mission* mission)
{
    auto route = new Route(mission->name() + " " + QObject::tr("route"));
    mission->setRoute(route);
    return route;
}

Waypoint* MavlinkMissionFactory::createWaypointForRoute(Route* route, WaypointType const* type)
{
    auto waypoint = new Waypoint(type);
    route->addWaypoint(waypoint);
    return waypoint;
}
