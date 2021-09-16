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

    m_waypoint->setType(type);

    // TODO: parameters to chain
    QVariantMap parameters;
    for (const Parameter& parameter : type->parameters)
    {
        if (parameter == mavlink_mission::latitude)
        {
            parameters.insert(parameter.name, item.x);
        }
        else if (parameter == mavlink_mission::longitude)
        {
            parameters.insert(parameter.name, item.y);
        }
        else if (parameter == mavlink_mission::altitude)
        {
            parameters.insert(parameter.name, item.z);
        }
        else if (parameter == mavlink_mission::abortAltitude)
        {
            parameters.insert(parameter.name, item.param1);
        }
        else if (parameter == mavlink_mission::relative)
        {
            parameters.insert(parameter.name, item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT);
        }
        else if (parameter == mavlink_mission::pitch)
        {
            parameters.insert(parameter.name, item.param1);
        }
        else if (parameter == mavlink_mission::radius)
        {
            parameters.insert(parameter.name, item.param3);
        }
        else if (parameter == mavlink_mission::loops)
        {
            parameters.insert(parameter.name, item.param1);
        }
        else if (parameter == mavlink_mission::clockwise)
        {
            parameters.insert(parameter.name, bool(item.param3 > 0));
        }
        else if (parameter == mavlink_mission::yaw)
        {
            parameters.insert(parameter.name, item.param4);
        }
    }
    m_waypoint->setParameters(parameters);
}

void MavlinkMissionWaypoint::fillMissionItem(mavlink_mission_item_int_t& item)
{
}
