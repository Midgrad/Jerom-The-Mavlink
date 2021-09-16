#ifndef MAVLINK_MISSION_TRAITS_H
#define MAVLINK_MISSION_TRAITS_H

#include "waypoint_type.h"

namespace md::domain::mavlink_mission
{
const QString missionType = "Mavlink mission";

const Parameter coordinate = { "Coordinate", Parameter::Coordinate };
const Parameter relative = { "Relative", Parameter::Bool, true };
const Parameter pitch = { "Pitch", Parameter::Real, 15 };
const Parameter radius = { "Radius", Parameter::Real, 100 };
const Parameter loops = { "Loops", Parameter::Int, 1 };
const Parameter clockwise = { "Clockwise", Parameter::Bool, true };

const WaypointType home = { "Home", { coordinate } };
const WaypointType waypoint = { "Waypoint", { coordinate, relative } };
const WaypointType takeoff = { "Takeoff", { coordinate, relative, pitch } };
const WaypointType landing = { "Landing", { coordinate, relative } };
const WaypointType loiterTurns = { "Loiter Turns",
                                   { coordinate, relative, radius, loops, clockwise } };

} // namespace md::domain::mavlink_mission

#endif // MAVLINK_MISSION_TRAITS_H
