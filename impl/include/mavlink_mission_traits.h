#ifndef MAVLINK_MISSION_TRAITS_H
#define MAVLINK_MISSION_TRAITS_H

#include "mission_traits.h"

namespace md::domain::mavlink_mission
{
const Parameter abortAltitude = { "abort_altitude", Parameter::Real, 50.0 };
const Parameter pitch = { "pitch", Parameter::Real, 15.0 };
const Parameter radius = { "radius", Parameter::Real, 100 };
const Parameter passRadius = { "pass_radius", Parameter::Real, 0 };
const Parameter loops = { "loops", Parameter::Int, 1 };
const Parameter time = { "time", Parameter::Int, 0 };
const Parameter clockwise = { "clockwise", Parameter::Bool, true };
const Parameter yaw = { "yaw", Parameter::Real, qQNaN() };

// TODO: remove home point with route reference altitude
const WaypointType home = { "home",
                            { &mission::latitude, &mission::longitude, &mission::altitude,
                              &mission::relative, &yaw } };
const WaypointType waypoint = { "waypoint",
                                { &mission::latitude, &mission::longitude, &mission::altitude,
                                  &mission::relative, &time, &radius, &passRadius, &yaw } };
const WaypointType takeoff = { "takeoff",
                               { &mission::latitude, &mission::longitude, &mission::altitude,
                                 &mission::relative, &pitch, &yaw } };
const WaypointType landing = { "landing",
                               { &mission::latitude, &mission::longitude, &mission::altitude,
                                 &mission::relative, &yaw, &abortAltitude } };
const WaypointType loiterTurns = { "loiter_turns",
                                   { &mission::latitude, &mission::longitude, &mission::altitude,
                                     &mission::relative, &radius, &loops, &clockwise, &yaw } };
const WaypointType loiterAlt = { "loiter_alt",
                                 { &mission::latitude, &mission::longitude, &mission::altitude,
                                   &mission::relative, &radius, &clockwise, &yaw } };

const RouteType routeType = { "mavlink_route",
                              { &waypoint, &takeoff, &landing, &loiterTurns, &loiterAlt } };
const MissionType missionType = { "mavlink_mission", &routeType, &home };

} // namespace md::domain::mavlink_mission

#endif // MAVLINK_MISSION_TRAITS_H
