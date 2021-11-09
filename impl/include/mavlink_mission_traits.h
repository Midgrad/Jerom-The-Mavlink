#ifndef MAVLINK_MISSION_TRAITS_H
#define MAVLINK_MISSION_TRAITS_H

#include "mission_traits.h"

namespace md::domain
{
namespace route
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
const RouteItemType home = { "home",
                             QT_TRANSLATE_NOOP("Route", "Home"),
                             QT_TRANSLATE_NOOP("Route", "HOME"),
                             { &latitude, &longitude, &altitude, &relativeAlt, &yaw } };
const RouteItemType waypoint = { "waypoint",
                                 QT_TRANSLATE_NOOP("Route", "Waypoint"),
                                 QT_TRANSLATE_NOOP("Route", "WPT"),
                                 { &latitude, &longitude, &altitude, &relativeAlt, &time, &radius,
                                   &passRadius, &yaw } };
// Takeoff & landing
const RouteItemType takeoff = { "takeoff",
                                QT_TRANSLATE_NOOP("Route", "Takeoff"),
                                QT_TRANSLATE_NOOP("Route", "TKFF"),
                                { &latitude, &longitude, &altitude, &relativeAlt, &pitch, &yaw } };
const RouteItemType landStart = { "landing_start",
                                  QT_TRANSLATE_NOOP("Route", "Landing start"),
                                  QT_TRANSLATE_NOOP("Route", "LND SRT"),
                                  {} };
const RouteItemType landing = { "landing",
                                QT_TRANSLATE_NOOP("Route", "Landing"),
                                QT_TRANSLATE_NOOP("Route", "LANDING"),
                                { &latitude, &longitude, &altitude, &relativeAlt, &yaw,
                                  &abortAltitude } };
// Loiters
const RouteItemType loiterTurns = { "loiter_turns",
                                    QT_TRANSLATE_NOOP("Route", "Loiter turns"),
                                    QT_TRANSLATE_NOOP("Route", "LTR TRN"),
                                    { &latitude, &longitude, &altitude, &relativeAlt, &radius,
                                      &clockwise, &turns, &headingRequired, &xtrack } };
const RouteItemType loiterAlt = { "loiter_alt",
                                  QT_TRANSLATE_NOOP("Route", "Loiter alt"),
                                  QT_TRANSLATE_NOOP("Route", "LTR ALT"),
                                  { &latitude, &longitude, &altitude, &relativeAlt, &radius,
                                    &clockwise, &headingRequired, &xtrack } };
const RouteItemType loiterUnlim = { "loiter_unlim",
                                    QT_TRANSLATE_NOOP("Route", "Loiter unlim"),
                                    QT_TRANSLATE_NOOP("Route", "LTR UNL"),
                                    { &latitude, &longitude, &altitude, &relativeAlt, &radius,
                                      &clockwise, &yaw } };
const RouteItemType loiterTime = { "loiter_time",
                                   QT_TRANSLATE_NOOP("Route", "Loiter time"),
                                   QT_TRANSLATE_NOOP("Route", "LTR TM"),
                                   { &latitude, &longitude, &altitude, &relativeAlt, &radius,
                                     &clockwise, &time, &headingRequired, &xtrack } };
// Payload control
const RouteItemType setTriggerDist = { "cam_tg_dist",
                                       QT_TRANSLATE_NOOP("Route", "Camera tg. dist."),
                                       QT_TRANSLATE_NOOP("Route", "CAM DST"),
                                       { &distance, &shutter, &trgOnce } };
const RouteItemType setTriggerInt = { "cam_tg_int",
                                      QT_TRANSLATE_NOOP("Route", "Camera tg. int"),
                                      QT_TRANSLATE_NOOP("Route", "CAM INT"),
                                      { &interval, &shutter } };

// Routes
const RouteType mavlinkRouteType = { "mavlink_route",
                                     QT_TRANSLATE_NOOP("Route", "Plain route (Mavlink)"),
                                     { &waypoint, &takeoff, &landStart, &landing, &loiterTurns,
                                       &loiterAlt, &loiterUnlim, &loiterTime, &setTriggerDist,
                                       &setTriggerInt } };
} // namespace route

// Mission
namespace mission
{
const MissionType mavlinkMissionType = { "mavlink_mission",
                                         QT_TRANSLATE_NOOP("Mission", "Mavlink Mission"),
                                         &route::mavlinkRouteType, &route::home };
}
} // namespace md::domain

#endif // MAVLINK_MISSION_TRAITS_H
