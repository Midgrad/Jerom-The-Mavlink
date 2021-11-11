#ifndef MAVLINK_MISSION_TRAITS_H
#define MAVLINK_MISSION_TRAITS_H

#include "mission_traits.h"

namespace md::domain
{
namespace route
{
// Params
const Parameter abortAltitude = { "abort_altitude", TR_ROUTE("Abort alt."),
                                  Parameter::Real,  50.0,
                                  minAltitude,      maxAltitude };
const Parameter passRadius = { "pass_radius", TR_ROUTE("Pass radius"), Parameter::Real, 0, 0,
                               maxRadius };
const Parameter turns = { "turns", TR_ROUTE("Turns"), Parameter::Int, 1, 1, 100 };
const Parameter clockwise = { "clockwise", TR_ROUTE("Clockwise"), true };
const Parameter headingRequired{ "heading_required", TR_ROUTE("Hdg req."), false };
const Parameter xtrack{ "xtrack", TR_ROUTE("XTrack"), Parameter::Real, qQNaN(), 0, 360 };
const Parameter shutter = { "shutter", TR_ROUTE("Shutter"), Parameter::Int, 0, 0, 1000 };
const Parameter trgOnce = { "trg_once", TR_ROUTE("Trigger"), false };
const Parameter interval = { "interval", TR_ROUTE("Interval"), Parameter::Int, 0, 0, 1000 };

// RouteItems
const RouteItemType setTriggerDist = { "mavlink_cam_tg_dist",
                                       TR_ROUTE("Camera tg. dist."),
                                       TR_ROUTE("CAM DST"),
                                       { &distance, &shutter, &trgOnce } };
const RouteItemType setTriggerInt = {
    "mavlink_cam_tg_int", TR_ROUTE("Camera tg. int"), TR_ROUTE("CAM INT"), { &interval, &shutter }
};

const RouteItemType landStart = {
    "mavlink_landing_start", TR_ROUTE("Landing start"), TR_ROUTE("LND SRT"), {}
};

// Waypoints
const WaypointType home = {
    "mavlink_home", TR_ROUTE("Home"), TR_ROUTE("HOME"), { &relativeAlt, &yaw }
};
const WaypointType waypoint = { "mavlink_waypoint",
                                TR_ROUTE("Waypoint"),
                                TR_ROUTE("WPT"),
                                { &relativeAlt, &time, &acceptRadius, &passRadius, &yaw },
                                { &setTriggerDist, &setTriggerInt, &landStart } };
const WaypointType takeoff = {
    "mavlink_takeoff", TR_ROUTE("Takeoff"), TR_ROUTE("TKFF"), { &relativeAlt, &pitch, &yaw }
};
const WaypointType landing = { "mavlink_landing",
                               TR_ROUTE("Landing"),
                               TR_ROUTE("LANDING"),
                               { &relativeAlt, &yaw, &abortAltitude } };
const WaypointType loiterTurns = { "mavlink_loiter_turns",
                                   TR_ROUTE("Loiter turns"),
                                   TR_ROUTE("LTR TRN"),
                                   { &relativeAlt, &radius, &clockwise, &turns, &headingRequired,
                                     &xtrack } };
const WaypointType loiterAlt = { "mavlink_loiter_alt",
                                 TR_ROUTE("Loiter alt"),
                                 TR_ROUTE("LTR ALT"),
                                 { &relativeAlt, &radius, &clockwise, &headingRequired, &xtrack } };
const WaypointType loiterUnlim = { "mavlink_loiter_unlim",
                                   TR_ROUTE("Loiter unlim"),
                                   TR_ROUTE("LTR UNL"),
                                   { &relativeAlt, &radius, &clockwise, &yaw } };
const WaypointType loiterTime = { "mavlink_loiter_time",
                                  TR_ROUTE("Loiter time"),
                                  TR_ROUTE("LTR TM"),
                                  { &relativeAlt, &radius, &clockwise, &time, &headingRequired,
                                    &xtrack } };

// Routes
const RouteType mavlinkRouteType = { "mavlink_route",
                                     TR_ROUTE("Plain route (Mavlink)"),
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
