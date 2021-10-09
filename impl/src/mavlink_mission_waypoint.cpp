#include "mavlink_mission_waypoint.h"

#include <QDebug>

#include "mavlink_mission_traits.h"
#include "mavlink_protocol_helpers.h"

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

void insertParameter(const Parameter& parameter, QVariantMap& parameters)
{
}
} // namespace

MavlinkMissionWaypoint::MavlinkMissionWaypoint(Waypoint* waypoint) : m_waypoint(waypoint)
{
    Q_ASSERT(waypoint);

    using ItemRef = const mavlink_mission_item_t&;

    m_waypointFillers.insert(mavlink_mission::latitude.name, [](ItemRef item) {
        return item.x;
    });
    m_waypointFillers.insert(mavlink_mission::longitude.name, [](ItemRef item) {
        return item.y;
    });
    m_waypointFillers.insert(mavlink_mission::altitude.name, [](ItemRef item) {
        return item.z;
    });
    m_waypointFillers.insert(mavlink_mission::abortAltitude.name, [](ItemRef item) {
        return item.param1;
    });
    m_waypointFillers.insert(mavlink_mission::relative.name, [](ItemRef item) {
        return item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT;
    });
    m_waypointFillers.insert(mavlink_mission::pitch.name, [](ItemRef item) {
        return item.param1;
    });
    m_waypointFillers.insert(mavlink_mission::radius.name, [](ItemRef item) {
        return item.param3;
    });
    m_waypointFillers.insert(mavlink_mission::loops.name, [](ItemRef item) {
        return item.param1;
    });
    m_waypointFillers.insert(mavlink_mission::clockwise.name, [](ItemRef item) {
        return item.param3 > 0;
    });
    m_waypointFillers.insert(mavlink_mission::yaw.name, [](ItemRef item) {
        return item.param4;
    });
}

void MavlinkMissionWaypoint::fillFromMissionItem(const mavlink_mission_item_t& item)
{
    const WaypointType* type = item.seq ? ::commandTypes.value(item.command)
                                        : &mavlink_mission::home;
    if (!type)
    {
        qWarning() << "Unsupported waypoint type" << type;
        return;
    }

    m_waypoint->setType(type);
    m_waypoint->setName(item.seq ? QObject::tr("WPT %1").arg(item.seq) : QObject::tr("HOME"));

    QVariantMap parameters;
    for (const Parameter& parameter : type->parameters)
    {
        if (!m_waypointFillers.contains(parameter.name))
            return;

        QVariant guarded = parameter.guard(m_waypointFillers[parameter.name](item));
        parameters.insert(parameter.name, guarded);
    }

    m_waypoint->setParameters(parameters);
}

void MavlinkMissionWaypoint::fillMissionItem(mavlink_mission_item_t& item)
{
    // TODO: fill mission Item
}
