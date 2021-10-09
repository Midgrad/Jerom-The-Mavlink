#ifndef MAVLINK_MISSION_TRAITS_H
#define MAVLINK_MISSION_TRAITS_H

#include "mission_type.h"

namespace md::domain::mavlink_mission
{
const Parameter latitude = { "latitude", Parameter::Real, 0.0 };
const Parameter longitude = { "longitude", Parameter::Real, 0.0 };
const Parameter altitude = { "altitude", Parameter::Real, 100.0 };
const Parameter abortAltitude = { "abort_altitude", Parameter::Real, 50.0 };
const Parameter relative = { "relative", Parameter::Bool, true };
const Parameter pitch = { "pitch", Parameter::Real, 15.0 };
const Parameter radius = { "radius", Parameter::Real, 100 };
const Parameter loops = { "loops", Parameter::Int, 1 };
const Parameter clockwise = { "clockwise", Parameter::Bool, true };
const Parameter yaw = { "yaw", Parameter::Real, 0.0 };

// TODO: remove home point with route reference altitude
const WaypointType home = { "home", { latitude, longitude, altitude } };
const WaypointType waypoint = { "waypoint", { latitude, longitude, altitude, relative } };
const WaypointType takeoff = { "takeoff", { latitude, longitude, altitude, relative, pitch } };
const WaypointType landing = { "landing",
                               { latitude, longitude, altitude, relative, yaw, abortAltitude } };
const WaypointType loiterTurns = {
    "loiter_turns", { latitude, longitude, altitude, relative, radius, loops, clockwise, yaw }
};

const RouteType routeType = { "mavlink_route", { &home, &waypoint, &takeoff, &landing } };
const MissionType missionType = { "mavlink_mission", &routeType };

} // namespace md::domain::mavlink_mission

#endif // MAVLINK_MISSION_TRAITS_H
