#ifndef MAVLINK_MISSION_TRAITS_H
#define MAVLINK_MISSION_TRAITS_H

#include "mission_traits.h"

namespace md::domain::mavlink_mission
{
const Parameter abortAltitude = { "abort_altitude", QT_TRANSLATE_NOOP("Route", "Abort alt."),
                                  Parameter::Real, 50.0 };
const Parameter pitch = { "pitch", QT_TRANSLATE_NOOP("Route", "Pitch"), Parameter::Real, 15.0 };
const Parameter radius = { "radius", QT_TRANSLATE_NOOP("Route", "Radius"), Parameter::Real, 100 };
const Parameter passRadius = { "pass_radius", QT_TRANSLATE_NOOP("Route", "Pass radius"),
                               Parameter::Real, 0 };
const Parameter loops = { "loops", QT_TRANSLATE_NOOP("Route", "Loops"), Parameter::Int, 1 };
const Parameter time = { "time", QT_TRANSLATE_NOOP("Route", "Time"), Parameter::Int, 0 };
const Parameter clockwise = { "clockwise", QT_TRANSLATE_NOOP("Route", "Clockwise"), Parameter::Bool,
                              true };
const Parameter yaw = { "yaw", QT_TRANSLATE_NOOP("Route", "Yaw"), Parameter::Real, qQNaN() };

// TODO: remove home point with route reference altitude
const WaypointType home = { "home",
                            QT_TRANSLATE_NOOP("Route", "HOME"),
                            { &mission::latitude, &mission::longitude, &mission::altitude,
                              &mission::relative, &yaw } };
const WaypointType waypoint = { "waypoint",
                                QT_TRANSLATE_NOOP("Route", "WAYPOINT"),
                                { &mission::latitude, &mission::longitude, &mission::altitude,
                                  &mission::relative, &time, &radius, &passRadius, &yaw } };
const WaypointType takeoff = { "takeoff",
                               QT_TRANSLATE_NOOP("Route", "TAKEOFF"),
                               { &mission::latitude, &mission::longitude, &mission::altitude,
                                 &mission::relative, &pitch, &yaw } };
const WaypointType landing = { "landing",
                               QT_TRANSLATE_NOOP("Route", "LANDING"),
                               { &mission::latitude, &mission::longitude, &mission::altitude,
                                 &mission::relative, &yaw, &abortAltitude } };
const WaypointType loiterTurns = { "loiter_turns",
                                   QT_TRANSLATE_NOOP("Route", "LOITER TURNS"),
                                   { &mission::latitude, &mission::longitude, &mission::altitude,
                                     &mission::relative, &radius, &loops, &clockwise, &yaw } };
const WaypointType loiterAlt = { "loiter_alt",
                                 QT_TRANSLATE_NOOP("Route", "LOITER ALT"),
                                 { &mission::latitude, &mission::longitude, &mission::altitude,
                                   &mission::relative, &radius, &clockwise, &yaw } };

const RouteType routeType = { "mavlink_route",
                              QT_TRANSLATE_NOOP("Route", "Plain route (Mavlink)"),
                              { &waypoint, &takeoff, &landing, &loiterTurns, &loiterAlt } };

const MissionType missionType = { "mavlink_mission",
                                  QT_TRANSLATE_NOOP("Mission", "Mavlink Mission"), &routeType,
                                  &home };

} // namespace md::domain::mavlink_mission

#endif // MAVLINK_MISSION_TRAITS_H
