#include "mavlink_item_convertors.h"

#include <QDebug>

#include "mavlink_mission_traits.h"
#include "mavlink_protocol_helpers.h"

using namespace md::domain;

namespace
{
const QMap<uint16_t, QString> commandTypes = {
    { MAV_CMD_DO_SET_HOME, mavlink_mission::home.name },
    { MAV_CMD_NAV_WAYPOINT, mavlink_mission::waypoint.name },
    { MAV_CMD_NAV_TAKEOFF, mavlink_mission::takeoff.name },
    { MAV_CMD_NAV_LAND, mavlink_mission::landing.name },
    { MAV_CMD_NAV_LOITER_TURNS, mavlink_mission::loiterTurns.name },
    { MAV_CMD_NAV_LOITER_TO_ALT, mavlink_mission::loiterAlt.name }
};
}

namespace md::domain
{
class PositionedConvertor : public IMavlinkItemConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setAndCheckParameter(mission::latitude.name, item.x);
        waypoint->setAndCheckParameter(mission::longitude.name, item.y);
        waypoint->setAndCheckParameter(mission::altitude.name, item.z);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.x = waypoint->parameter(mission::latitude.name).toReal();
        item.y = waypoint->parameter(mission::longitude.name).toReal();
        item.z = waypoint->parameter(mission::altitude.name).toReal();
    }
};

class HomeConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mavlink_mission::home);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mavlink_mission::yaw.name, item.param4);
        // NOTE: current/specified position
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_HOME;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.frame = MAV_FRAME_GLOBAL;
        item.param1 = 0;
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mavlink_mission::yaw.name).toReal();
    }
};

class WaypointConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mavlink_mission::waypoint);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mission::relative.name,
                                       item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT);
        waypoint->setAndCheckParameter(mavlink_mission::time.name, item.param1);
        waypoint->setAndCheckParameter(mavlink_mission::radius.name, item.param2);
        waypoint->setAndCheckParameter(mavlink_mission::passRadius.name, item.param3);
        waypoint->setAndCheckParameter(mavlink_mission::yaw.name, item.param4);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_WAYPOINT;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.frame = waypoint->parameter(mission::relative.name).toBool()
                         ? MAV_FRAME_GLOBAL_RELATIVE_ALT
                         : MAV_FRAME_GLOBAL;
        item.param1 = waypoint->parameter(mavlink_mission::time.name).toInt();
        item.param2 = waypoint->parameter(mavlink_mission::radius.name).toReal();
        item.param3 = waypoint->parameter(mavlink_mission::passRadius.name).toReal();
        item.param4 = waypoint->parameter(mavlink_mission::yaw.name).toReal();
    }
};

class TakeoffConvertor : public PositionedConvertor
{
public:
    /* Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.
     * Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  */
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mavlink_mission::takeoff);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mavlink_mission::pitch.name, item.param1);
        waypoint->setAndCheckParameter(mavlink_mission::yaw.name, item.param4);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_TAKEOFF;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mavlink_mission::pitch.name).toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mavlink_mission::yaw.name).toReal();
    }
};

class LandingConvertor : public WaypointConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mavlink_mission::landing);
        WaypointConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mavlink_mission::abortAltitude.name, item.param1);
        waypoint->setAndCheckParameter(mavlink_mission::yaw.name, item.param4);
        // NOTE: Precision land mode
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LAND;
        WaypointConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mavlink_mission::abortAltitude.name).toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mavlink_mission::yaw.name).toReal();
    }
};

class LoiterTurnsConvertor : public WaypointConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mavlink_mission::loiterTurns);
        WaypointConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mavlink_mission::loops.name, item.param1);
        waypoint->setAndCheckParameter(mavlink_mission::clockwise.name, item.param3 > 0);
        waypoint->setAndCheckParameter(mavlink_mission::radius.name, item.param3);
        // NOTE:  Heading Required, Xtrack Location
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TURNS;
        WaypointConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mavlink_mission::loops.name).toInt();
        item.param2 = 0;
        auto radius = waypoint->parameter(mavlink_mission::radius.name).toReal();
        item.param3 = waypoint->parameter(mavlink_mission::clockwise.name).toBool() ? radius
                                                                                    : -radius;
        item.param4 = 0;
    }
};

class LoiterAltConvertor : public WaypointConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mavlink_mission::loiterAlt);
        WaypointConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mavlink_mission::clockwise.name, item.param2 > 0);
        waypoint->setAndCheckParameter(mavlink_mission::radius.name, item.param2);
        // NOTE: Heading Required, Xtrack Location
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TO_ALT;
        WaypointConvertor::waypointToItem(waypoint, item);
        item.param1 = 0;
        auto radius = waypoint->parameter(mavlink_mission::radius.name).toReal();
        item.param2 = waypoint->parameter(mavlink_mission::clockwise.name).toBool() ? radius
                                                                                    : -radius;
        item.param3 = 0;
        item.param4 = 0;
    }
};
} // namespace md::domain

MavlinkItemConvertorsPool::MavlinkItemConvertorsPool() :
    m_convertors({
        { mavlink_mission::home.name, new HomeConvertor() },
        { mavlink_mission::waypoint.name, new WaypointConvertor() },
        { mavlink_mission::takeoff.name, new TakeoffConvertor() },
        { mavlink_mission::landing.name, new LandingConvertor() },
        { mavlink_mission::loiterTurns.name, new LoiterTurnsConvertor() },
        { mavlink_mission::loiterAlt.name, new LoiterAltConvertor() },
    })
{
}

MavlinkItemConvertorsPool::~MavlinkItemConvertorsPool()
{
    qDeleteAll(m_convertors);
}

IMavlinkItemConvertor* MavlinkItemConvertorsPool::convertor(const WaypointType* type)
{
    return m_convertors.value(type->name, nullptr);
}

IMavlinkItemConvertor* MavlinkItemConvertorsPool::convertor(uint16_t commandType)
{
    return m_convertors.value(::commandTypes.value(commandType, nullptr), nullptr);
}
