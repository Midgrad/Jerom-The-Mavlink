#include "mavlink_mission_waypoint.h"

#include <QDebug>

#include "mavlink_mission_traits.h"

using namespace md::domain;

namespace
{
const QMap<uint16_t, const WaypointType*> commandTypes = {
    { MAV_CMD_DO_SET_HOME, &mavlink_mission::home },
    { MAV_CMD_NAV_WAYPOINT, &mavlink_mission::waypoint },
    { MAV_CMD_NAV_TAKEOFF, &mavlink_mission::takeoff },
    { MAV_CMD_NAV_LAND, &mavlink_mission::landing },
    { MAV_CMD_NAV_LOITER_TURNS, &mavlink_mission::loiterTurns }
};
}

MavlinkMissionWaypoint::MavlinkMissionWaypoint(Waypoint* waypoint) : m_waypoint(waypoint)
{
    Q_ASSERT(waypoint);
}

void MavlinkMissionWaypoint::fillFromMissionItem(const mavlink_mission_item_int_t& item)
{
    const WaypointType* type = ::commandTypes.value(item.command);
    if (!type)
    {
        qWarning() << "Unsupported waypoint type" << type;
        return;
    }

    //m_waypoint->setType(type);
}

void MavlinkMissionWaypoint::fillMissionItem(mavlink_mission_item_int_t& item)
{
}
