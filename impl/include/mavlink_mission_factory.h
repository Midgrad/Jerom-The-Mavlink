#ifndef MAVLINK_MISSION_FACTORY_H
#define MAVLINK_MISSION_FACTORY_H

#include "i_mission_factory.h"
#include "mavlink_mission_traits.h"

namespace md::domain
{
class MavlinkMissionFactory : public IMissionFactory
{
public:
    MavlinkMissionFactory();

    Mission* createMission() override;

    Route* createRouteForMission(Mission* mission);
    Waypoint* createWaypointForRoute(Route* route);
};
} // namespace md::domain

#endif // MAVLINK_MISSION_FACTORY_H
