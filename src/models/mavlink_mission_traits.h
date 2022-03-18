#ifndef MAVLINK_MISSION_TRAITS_H
#define MAVLINK_MISSION_TRAITS_H

#include "mission_traits.h"

namespace md::domain
{
namespace mission
{
// Parameters
const ParameterType abortAltitude = { "abort_altitude",    TR_MISSION("Abort alt."),
                                      ParameterType::Real, 50.0,
                                      minAltitude,         maxAltitude };
const ParameterType passRadius = {
    "pass_radius", TR_MISSION("Pass radius"), ParameterType::Real, 0, 0, maxRadius
};
const ParameterType turns = { "turns", TR_MISSION("Turns"), ParameterType::Int, 1, 1, 100 };
const ParameterType clockwise = { "clockwise", TR_MISSION("Clockwise"), true };
const ParameterType headingRequired{ "heading_required", TR_MISSION("Hdg req."), false };
const ParameterType xtrack{ "xtrack", TR_MISSION("XTrack"), ParameterType::Real, 0, 0, 360 };
const ParameterType shutter = { "shutter", TR_MISSION("Shutter"), ParameterType::Int, 0, 0, 1000 };
const ParameterType trgOnce = { "trg_once", TR_MISSION("Trigger"), false };
const ParameterType interval = { "interval", TR_MISSION("Interval"), ParameterType::Int, 0, 0, 1000 };

// Mission Items
const MissionItemType home = {
    "mavlink_home", TR_MISSION("Home"), TR_MISSION("HOME"), Positioned::Required, { &yaw }
};
const MissionItemType waypoint = { "mavlink_waypoint",
                                   TR_MISSION("Waypoint"),
                                   TR_MISSION("WPT"),
                                   Positioned::Required,
                                   { &time, &acceptRadius, &passRadius, &yaw } };
const MissionItemType takeoff = {
    "mavlink_takeoff", TR_MISSION("Takeoff"), TR_MISSION("TKFF"), Positioned::Required, { &pitch, &yaw }
};
const MissionItemType landing = { "mavlink_landing",
                                  TR_MISSION("Landing"),
                                  TR_MISSION("LANDING"),
                                  Positioned::Required,
                                  { &yaw, &abortAltitude } };
const MissionItemType loiterTurns = { "mavlink_loiter_turns",
                                      TR_MISSION("Loiter turns"),
                                      TR_MISSION("LTR TRN"),
                                      Positioned::Required,
                                      { &radius, &clockwise, &turns, &headingRequired, &xtrack } };
const MissionItemType loiterAlt = { "mavlink_loiter_alt",
                                    TR_MISSION("Loiter alt"),
                                    TR_MISSION("LTR ALT"),
                                    Positioned::Required,
                                    { &radius, &clockwise, &headingRequired, &xtrack } };
const MissionItemType loiterUnlim = { "mavlink_loiter_unlim",
                                      TR_MISSION("Loiter unlim"),
                                      TR_MISSION("LTR UNL"),
                                      Positioned::Required,
                                      { &radius, &clockwise, &yaw } };
const MissionItemType loiterTime = { "mavlink_loiter_time",
                                     TR_MISSION("Loiter time"),
                                     TR_MISSION("LTR TM"),
                                     Positioned::Required,
                                     { &radius, &clockwise, &time, &headingRequired, &xtrack } };

const MissionItemType setTriggerDist = { "mavlink_cam_tg_dist",
                                         TR_MISSION("Camera tg. dist."),
                                         TR_MISSION("CAM DST"),
                                         Positioned::No,
                                         { &distance, &shutter, &trgOnce } };
const MissionItemType setTriggerInt = { "mavlink_cam_tg_int",
                                        TR_MISSION("Camera tg. int"),
                                        TR_MISSION("CAM INT"),
                                        Positioned::No,
                                        { &interval, &shutter } };

const MissionItemType landStart = { "mavlink_landing_start", TR_MISSION("Landing start"),
                                    TR_MISSION("LND SRT"), Positioned::Optional };

// Route pattens
const ParameterType surveyType = { "survey_type",
                                   TR_MISSION("Survey type"),
                                   { TR_MISSION("None"), setTriggerDist.name, setTriggerInt.name },
                                   TR_MISSION("None") };

const RoutePatternType surveyPatternGrid = { "mavlink_survey_grid",
                                             TR_MISSION("Survey grid"),
                                             "qrc:/icons/grid_pattern.svg",
                                             { &surveyType, &altitude, &heading, &spacing,
                                               &doubled } };
const RoutePatternType surveyPatternSnail = { "mavlink_survey_snail",
                                              TR_MISSION("Survey snail"),
                                              "qrc:/icons/snail_pattern.svg",
                                              { &surveyType, &altitude, &heading, &spacing } };

// Mission
const MissionType mavlinkMission = { "mavlink_mission",
                                     TR_MISSION("Mavlink Mission"),
                                     { &mission::home, &waypoint, &takeoff, &landing, &loiterTurns,
                                       &loiterAlt, &loiterUnlim, &loiterTime, &setTriggerDist,
                                       &setTriggerInt, &landStart },
                                     { &surveyPatternGrid, &surveyPatternSnail } };
} // namespace mission
} // namespace md::domain

#endif // MAVLINK_MISSION_TRAITS_H
