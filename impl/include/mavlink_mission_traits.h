#ifndef MAVLINK_MISSION_TRAITS_H
#define MAVLINK_MISSION_TRAITS_H

#include "waypoint_type.h"

namespace md::domain::mavlink_mission
{
const QString missionType = "Mavlink mission";

const Parameter latitude = { "Latitude", Parameter::Real, 0.0 };
const Parameter longitude = { "Longitude", Parameter::Real, 0.0 };
const Parameter altitude = { "Altitude", Parameter::Real, 100.0 };
const Parameter abortAltitude = { "Abort Altitude", Parameter::Real, 50.0 };
const Parameter relative = { "Relative", Parameter::Bool, true };
const Parameter pitch = { "Pitch", Parameter::Real, 15.0 };
const Parameter radius = { "Radius", Parameter::Real, 100 };
const Parameter loops = { "Loops", Parameter::Int, 1 };
const Parameter clockwise = { "Clockwise", Parameter::Bool, true };
const Parameter yaw = { "Yaw", Parameter::Real, 0.0 };

// TODO: remove home point with route reference altitude
const WaypointType home = { "Home", { latitude, longitude, altitude } };
const WaypointType waypoint = { "Waypoint", { latitude, longitude, altitude, relative } };
const WaypointType takeoff = { "Takeoff", { latitude, longitude, altitude, relative, pitch } };
const WaypointType landing = { "Landing",
                               { latitude, longitude, altitude, relative, yaw, abortAltitude } };
const WaypointType loiterTurns = {
    "Loiter Turns", { latitude, longitude, altitude, relative, radius, loops, clockwise, yaw }
};
} // namespace md::domain::mavlink_mission

#endif // MAVLINK_MISSION_TRAITS_H
