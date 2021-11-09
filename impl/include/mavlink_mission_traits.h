#ifndef MAVLINK_MISSION_TRAITS_H
#define MAVLINK_MISSION_TRAITS_H

#include "mission_traits.h"

namespace md::domain::mission
{
// Params
const Parameter abortAltitude = { "abort_altitude", QT_TRANSLATE_NOOP("Route", "Abort alt."),
                                  Parameter::Real, 50.0 };
const Parameter passRadius = { "pass_radius", QT_TRANSLATE_NOOP("Route", "Pass radius"),
                               Parameter::Real, 0 };
const Parameter turns = { "turns", QT_TRANSLATE_NOOP("Route", "Turns"), Parameter::Int, 1 };
const Parameter clockwise = { "clockwise", QT_TRANSLATE_NOOP("Route", "Clockwise"), Parameter::Bool,
                              true };
const Parameter headingRequired{ "heading_required", QT_TRANSLATE_NOOP("Route", "Hdg req."),
                                 Parameter::Bool, false };
const Parameter xtrack{ "xtrack", QT_TRANSLATE_NOOP("Route", "XTrack"), Parameter::Real, qQNaN(), 0,
                        360 };
const Parameter shutter = { "shutter", QT_TRANSLATE_NOOP("Route", "Shutter"), Parameter::Int, 0 };
const Parameter trgOnce = { "trg_once", QT_TRANSLATE_NOOP("Route", "Trigger"), Parameter::Bool,
                            false };
const Parameter interval = { "interval", QT_TRANSLATE_NOOP("Route", "Interval"), Parameter::Int, 0 };

// Basic
const WaypointType home = { "home",
                            QT_TRANSLATE_NOOP("Route", "Home"),
                            QT_TRANSLATE_NOOP("Route", "HOME"),
                            { &latitude, &longitude, &altitude, &relativeAlt, &yaw } };
const WaypointType waypoint = { "waypoint",
                                QT_TRANSLATE_NOOP("Route", "Waypoint"),
                                QT_TRANSLATE_NOOP("Route", "WPT"),
                                { &latitude, &longitude, &altitude, &relativeAlt, &time, &radius,
                                  &passRadius, &yaw } };
// Takeoff & landing
const WaypointType takeoff = { "takeoff",
                               QT_TRANSLATE_NOOP("Route", "Takeoff"),
                               QT_TRANSLATE_NOOP("Route", "TKFF"),
                               { &latitude, &longitude, &altitude, &relativeAlt, &pitch, &yaw } };
const WaypointType landStart = { "landing_start",
                                 QT_TRANSLATE_NOOP("Route", "Landing start"),
                                 QT_TRANSLATE_NOOP("Route", "LND SRT"),
                                 {} };
const WaypointType landing = { "landing",
                               QT_TRANSLATE_NOOP("Route", "Landing"),
                               QT_TRANSLATE_NOOP("Route", "LANDING"),
                               { &latitude, &longitude, &altitude, &relativeAlt, &yaw,
                                 &abortAltitude } };
// Loiters
const WaypointType loiterTurns = { "loiter_turns",
                                   QT_TRANSLATE_NOOP("Route", "Loiter turns"),
                                   QT_TRANSLATE_NOOP("Route", "LTR TRN"),
                                   { &latitude, &longitude, &altitude, &relativeAlt, &radius,
                                     &clockwise, &turns, &headingRequired, &xtrack } };
const WaypointType loiterAlt = { "loiter_alt",
                                 QT_TRANSLATE_NOOP("Route", "Loiter alt"),
                                 QT_TRANSLATE_NOOP("Route", "LTR ALT"),
                                 { &latitude, &longitude, &altitude, &relativeAlt, &radius,
                                   &clockwise, &headingRequired, &xtrack } };
const WaypointType loiterUnlim = { "loiter_unlim",
                                   QT_TRANSLATE_NOOP("Route", "Loiter unlim"),
                                   QT_TRANSLATE_NOOP("Route", "LTR UNL"),
                                   { &latitude, &longitude, &altitude, &relativeAlt, &radius,
                                     &clockwise, &yaw } };
const WaypointType loiterTime = { "loiter_time",
                                  QT_TRANSLATE_NOOP("Route", "Loiter time"),
                                  QT_TRANSLATE_NOOP("Route", "LTR TM"),
                                  { &latitude, &longitude, &altitude, &relativeAlt, &radius,
                                    &clockwise, &time, &headingRequired, &xtrack } };
// Payload control
const WaypointType setTriggerDist = { "cam_tg_dist",
                                      QT_TRANSLATE_NOOP("Route", "Camera tg. dist."),
                                      QT_TRANSLATE_NOOP("Route", "CAM DST"),
                                      { &distance, &shutter, &trgOnce } };
const WaypointType setTriggerInt = { "cam_tg_int",
                                     QT_TRANSLATE_NOOP("Route", "Camera tg. int"),
                                     QT_TRANSLATE_NOOP("Route", "CAM INT"),
                                     { &interval, &shutter } };

// Routes
const RouteType routeType = { "mavlink_route",
                              QT_TRANSLATE_NOOP("Route", "Plain route (Mavlink)"),
                              { &waypoint, &takeoff, &landStart, &landing, &loiterTurns, &loiterAlt,
                                &loiterUnlim, &loiterTime, &setTriggerDist, &setTriggerInt } };
// Mission

const MissionType missionType = { "mavlink_mission",
                                  QT_TRANSLATE_NOOP("Mission", "Mavlink Mission"), &routeType,
                                  &home };

} // namespace md::domain::mission

#endif // MAVLINK_MISSION_TRAITS_H
