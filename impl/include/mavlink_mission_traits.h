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

// RouteItems
const RouteItemType setTriggerDist = { "mavlink_cam_tg_dist",
                                       QT_TRANSLATE_NOOP("Route", "Camera tg. dist."),
                                       QT_TRANSLATE_NOOP("Route", "CAM DST"),
                                       { &distance, &shutter, &trgOnce } };
const RouteItemType setTriggerInt = { "mavlink_cam_tg_int",
                                      QT_TRANSLATE_NOOP("Route", "Camera tg. int"),
                                      QT_TRANSLATE_NOOP("Route", "CAM INT"),
                                      { &interval, &shutter } };

const RouteItemType landStart = { "mavlink_landing_start",
                                  QT_TRANSLATE_NOOP("Route", "Landing start"),
                                  QT_TRANSLATE_NOOP("Route", "LND SRT"),
                                  {} };

// Waypoints
const WaypointType home = { "mavlink_home",
                            QT_TRANSLATE_NOOP("Route", "Home"),
                            QT_TRANSLATE_NOOP("Route", "HOME"),
                            { &relativeAlt, &yaw } };
const WaypointType waypoint = { "mavlink_waypoint",
                                QT_TRANSLATE_NOOP("Route", "Waypoint"),
                                QT_TRANSLATE_NOOP("Route", "WPT"),
                                { &relativeAlt, &time, &radius, &passRadius, &yaw },
                                { &setTriggerDist, &setTriggerInt, &landStart } };
const WaypointType takeoff = { "mavlink_takeoff",
                               QT_TRANSLATE_NOOP("Route", "Takeoff"),
                               QT_TRANSLATE_NOOP("Route", "TKFF"),
                               { &relativeAlt, &pitch, &yaw } };
const WaypointType landing = { "mavlink_landing",
                               QT_TRANSLATE_NOOP("Route", "Landing"),
                               QT_TRANSLATE_NOOP("Route", "LANDING"),
                               { &relativeAlt, &yaw, &abortAltitude } };
const WaypointType loiterTurns = { "mavlink_loiter_turns",
                                   QT_TRANSLATE_NOOP("Route", "Loiter turns"),
                                   QT_TRANSLATE_NOOP("Route", "LTR TRN"),
                                   { &relativeAlt, &radius, &clockwise, &turns, &headingRequired,
                                     &xtrack } };
const WaypointType loiterAlt = { "mavlink_loiter_alt",
                                 QT_TRANSLATE_NOOP("Route", "Loiter alt"),
                                 QT_TRANSLATE_NOOP("Route", "LTR ALT"),
                                 { &relativeAlt, &radius, &clockwise, &headingRequired, &xtrack } };
const WaypointType loiterUnlim = { "mavlink_loiter_unlim",
                                   QT_TRANSLATE_NOOP("Route", "Loiter unlim"),
                                   QT_TRANSLATE_NOOP("Route", "LTR UNL"),
                                   { &relativeAlt, &radius, &clockwise, &yaw } };
const WaypointType loiterTime = { "mavlink_loiter_time",
                                  QT_TRANSLATE_NOOP("Route", "Loiter time"),
                                  QT_TRANSLATE_NOOP("Route", "LTR TM"),
                                  { &relativeAlt, &radius, &clockwise, &time, &headingRequired,
                                    &xtrack } };

// Routes
const RouteType mavlinkRouteType = { "mavlink_route",
                                     QT_TRANSLATE_NOOP("Route", "Plain route (Mavlink)"),
                                     { &waypoint, &takeoff, &landing, &loiterTurns, &loiterAlt,
                                       &loiterUnlim, &loiterTime } };
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
