#ifndef MAVLINK_MISSION_TRAITS_H
#define MAVLINK_MISSION_TRAITS_H

#include "mission_traits.h"

namespace md::domain::mission
{
// Parameters
constexpr char abortAltitude[] = "abortAltitude";
constexpr char passRadius[] = "passRadius";
constexpr char turns[] = "turns";
constexpr char clockwise[] = "clockwise";
constexpr char headingRequired[] = "headingRequired";
constexpr char xtrack[] = "xtrack";

constexpr char shutter[] = "shutter";
constexpr char trgOnce[] = "trgOnce";
constexpr char interval[] = "interval";

// Mission item types
constexpr char mavlinkWaypoint[] = "mavlink_waypoint";
constexpr char mavlinkHome[] = "mavlink_home";

constexpr char mavlinkTakeoff[] = "mavlink_takeoff";
constexpr char mavlinkLanding[] = "mavlink_landing";
constexpr char mavlinkLoiterTurns[] = "mavlink_loiterTurns";
constexpr char mavlinkLoiterAlt[] = "mavlink_loiterAlt";
constexpr char mavlinkLoiterUnlim[] = "mavlink_loiterUnlim";
constexpr char mavlinkLoiterTime[] = "mavlink_loiterTime";

constexpr char mavlinkSetTriggerDist[] = "mavlink_setTriggerDist";
constexpr char mavlinkSetTriggerInt[] = "mavlink_setTriggerInt";

constexpr char mavlinkLandStart[] = "mavlink_land_start";

// Mission type
constexpr char mavlinkMission[] = "mavlink_mission";

} // namespace md::domain::mission

#endif // MAVLINK_MISSION_TRAITS_H
