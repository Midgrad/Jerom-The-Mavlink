#ifndef MAVLINK_MISSION_TRAITS_H
#define MAVLINK_MISSION_TRAITS_H

#include "mission_traits.h"

namespace md::domain
{
namespace route
{
// Params
const ParameterType abortAltitude = { "abort_altitude",    TR_ROUTE("Abort alt."),
                                      ParameterType::Real, 50.0,
                                      minAltitude,         maxAltitude };
const ParameterType passRadius = {
    "pass_radius", TR_ROUTE("Pass radius"), ParameterType::Real, 0, 0, maxRadius
};
const ParameterType turns = { "turns", TR_ROUTE("Turns"), ParameterType::Int, 1, 1, 100 };
const ParameterType clockwise = { "clockwise", TR_ROUTE("Clockwise"), true };
const ParameterType headingRequired{ "heading_required", TR_ROUTE("Hdg req."), false };
const ParameterType xtrack{ "xtrack", TR_ROUTE("XTrack"), ParameterType::Real, 0, 0, 360 };
const ParameterType shutter = { "shutter", TR_ROUTE("Shutter"), ParameterType::Int, 0, 0, 1000 };
const ParameterType trgOnce = { "trg_once", TR_ROUTE("Trigger"), false };
const ParameterType interval = { "interval", TR_ROUTE("Interval"), ParameterType::Int, 0, 0, 1000 };

// Route Items
const RouteItemType home = {
    "mavlink_home", TR_ROUTE("Home"), TR_ROUTE("HOME"), Positioned::Required, { &yaw }
};
const RouteItemType waypoint = { "mavlink_waypoint",
                                 TR_ROUTE("Waypoint"),
                                 TR_ROUTE("WPT"),
                                 Positioned::Required,
                                 { &time, &acceptRadius, &passRadius, &yaw } };
const RouteItemType takeoff = {
    "mavlink_takeoff", TR_ROUTE("Takeoff"), TR_ROUTE("TKFF"), Positioned::Required, { &pitch, &yaw }
};
const RouteItemType landing = { "mavlink_landing",
                                TR_ROUTE("Landing"),
                                TR_ROUTE("LANDING"),
                                Positioned::Required,
                                { &yaw, &abortAltitude } };
const RouteItemType loiterTurns = { "mavlink_loiter_turns",
                                    TR_ROUTE("Loiter turns"),
                                    TR_ROUTE("LTR TRN"),
                                    Positioned::Required,
                                    { &radius, &clockwise, &turns, &headingRequired, &xtrack } };
const RouteItemType loiterAlt = { "mavlink_loiter_alt",
                                  TR_ROUTE("Loiter alt"),
                                  TR_ROUTE("LTR ALT"),
                                  Positioned::Required,
                                  { &radius, &clockwise, &headingRequired, &xtrack } };
const RouteItemType loiterUnlim = { "mavlink_loiter_unlim",
                                    TR_ROUTE("Loiter unlim"),
                                    TR_ROUTE("LTR UNL"),
                                    Positioned::Required,
                                    { &radius, &clockwise, &yaw } };
const RouteItemType loiterTime = { "mavlink_loiter_time",
                                   TR_ROUTE("Loiter time"),
                                   TR_ROUTE("LTR TM"),
                                   Positioned::Required,
                                   { &radius, &clockwise, &time, &headingRequired, &xtrack } };

const RouteItemType setTriggerDist = { "mavlink_cam_tg_dist",
                                       TR_ROUTE("Camera tg. dist."),
                                       TR_ROUTE("CAM DST"),
                                       Positioned::No,
                                       { &distance, &shutter, &trgOnce } };
const RouteItemType setTriggerInt = { "mavlink_cam_tg_int",
                                      TR_ROUTE("Camera tg. int"),
                                      TR_ROUTE("CAM INT"),
                                      Positioned::No,
                                      { &interval, &shutter } };

const RouteItemType landStart = { "mavlink_landing_start", TR_ROUTE("Landing start"),
                                  TR_ROUTE("LND SRT"), Positioned::Optional };

// Route pattens
const ParameterType surveyType = { "survey_type",
                                   TR_ROUTE("Survey type"),
                                   { TR_ROUTE("None"), setTriggerDist.name, setTriggerInt.name },
                                   TR_ROUTE("None") };

const RoutePatternType surveyPatternGrid = { "mavlink_survey_grid",
                                             TR_ROUTE("Survey grid"),
                                             "qrc:/icons/grid_pattern.svg",
                                             { &surveyType, &altitude, &heading, &spacing,
                                               &doubled } };
const RoutePatternType surveyPatternSnail = { "mavlink_survey_snail",
                                              TR_ROUTE("Survey snail"),
                                              "qrc:/icons/snail_pattern.svg",
                                              { &surveyType, &altitude, &heading, &spacing } };

// Routes
const RouteType mavlinkRouteType = { "mavlink_route",
                                     TR_ROUTE("Plain route (Mavlink)"),
                                     { &waypoint, &takeoff, &landing, &loiterTurns, &loiterAlt,
                                       &loiterUnlim, &loiterTime, &setTriggerDist, &setTriggerInt,
                                       &landStart },
                                     { &surveyPatternGrid, &surveyPatternSnail } };
} // namespace route

// Mission
namespace mission
{
const MissionType mavlinkMissionType = { "mavlink_mission",
                                         QT_TRANSLATE_NOOP("Mission", "Mavlink Mission"),
                                         &route::mavlinkRouteType, &route::home, &route::waypoint };
}
} // namespace md::domain

#endif // MAVLINK_MISSION_TRAITS_H
