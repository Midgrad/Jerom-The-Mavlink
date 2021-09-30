#ifndef MAVLINK_MISSION_WAYPOINT_H
#define MAVLINK_MISSION_WAYPOINT_H

#include <common/mavlink.h>

#include "waypoint.h"

namespace md::domain
{
class MavlinkMissionWaypoint
{
public:
    MavlinkMissionWaypoint(Waypoint* waypoint);

    void fillFromMissionItem(const mavlink_mission_item_t& item);
    void fillMissionItem(mavlink_mission_item_t& item);

private:
    Waypoint* const m_waypoint;

    QMap<QString, std::function<QVariant(const mavlink_mission_item_t& item)>> m_waypointFillers;
};

} // namespace md::domain

#endif // MAVLINK_MISSION_WAYPOINT_H
