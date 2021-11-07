#include "mavlink_item_convertors.h"

#include <QDebug>

#include "mavlink_mission_traits.h"
#include "mavlink_protocol_helpers.h"

using namespace md::domain;

namespace
{
const QMap<uint16_t, QString> commandTypes = {
    { MAV_CMD_DO_SET_HOME, mavlink_mission::home.id },
    { MAV_CMD_NAV_WAYPOINT, mavlink_mission::waypoint.id },
    { MAV_CMD_NAV_TAKEOFF, mavlink_mission::takeoff.id },
    { MAV_CMD_NAV_LAND, mavlink_mission::landing.id },
    { MAV_CMD_NAV_LOITER_TURNS, mavlink_mission::loiterTurns.id },
    { MAV_CMD_NAV_LOITER_TO_ALT, mavlink_mission::loiterAlt.id }
};
}

namespace md::domain
{
class PositionedConvertor : public IMavlinkItemConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setAndCheckParameter(mission::relative.id,
                                       item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT);
        waypoint->setAndCheckParameter(mission::latitude.id, item.x);
        waypoint->setAndCheckParameter(mission::longitude.id, item.y);
        waypoint->setAndCheckParameter(mission::altitude.id, item.z);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.frame = waypoint->parameter(mission::relative.id).toBool()
                         ? MAV_FRAME_GLOBAL_RELATIVE_ALT
                         : MAV_FRAME_GLOBAL;
        item.x = waypoint->parameter(mission::latitude.id).toReal();
        item.y = waypoint->parameter(mission::longitude.id).toReal();
        item.z = waypoint->parameter(mission::altitude.id).toReal();
    }
};

class HomeConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mavlink_mission::home);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mavlink_mission::yaw.id, item.param4);
        // NOTE: current/specified position
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_HOME;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = 0;
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mavlink_mission::yaw.id).toReal();
    }
};

class WaypointConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mavlink_mission::waypoint);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mavlink_mission::time.id, item.param1);
        waypoint->setAndCheckParameter(mavlink_mission::radius.id, item.param2);
        waypoint->setAndCheckParameter(mavlink_mission::passRadius.id, item.param3);
        waypoint->setAndCheckParameter(mavlink_mission::yaw.id, item.param4);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_WAYPOINT;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mavlink_mission::time.id).toInt();
        item.param2 = waypoint->parameter(mavlink_mission::radius.id).toReal();
        item.param3 = waypoint->parameter(mavlink_mission::passRadius.id).toReal();
        item.param4 = waypoint->parameter(mavlink_mission::yaw.id).toReal();
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
        waypoint->setAndCheckParameter(mavlink_mission::pitch.id, item.param1);
        waypoint->setAndCheckParameter(mavlink_mission::yaw.id, item.param4);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_TAKEOFF;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mavlink_mission::pitch.id).toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mavlink_mission::yaw.id).toReal();
    }
};

class LandingConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mavlink_mission::landing);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mavlink_mission::abortAltitude.id, item.param1);
        waypoint->setAndCheckParameter(mavlink_mission::yaw.id, item.param4);
        // NOTE: Precision land mode
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LAND;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mavlink_mission::abortAltitude.id).toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mavlink_mission::yaw.id).toReal();
    }
};

class LoiterTurnsConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mavlink_mission::loiterTurns);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mavlink_mission::loops.id, item.param1);
        waypoint->setAndCheckParameter(mavlink_mission::clockwise.id, item.param3 > 0);
        waypoint->setAndCheckParameter(mavlink_mission::radius.id, item.param3);
        // NOTE:  Heading Required, Xtrack Location
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TURNS;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mavlink_mission::loops.id).toInt();
        item.param2 = 0;
        auto radius = waypoint->parameter(mavlink_mission::radius.id).toReal();
        item.param3 = waypoint->parameter(mavlink_mission::clockwise.id).toBool() ? radius
                                                                                  : -radius;
        item.param4 = 0;
    }
};

class LoiterAltConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mavlink_mission::loiterAlt);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mavlink_mission::clockwise.id, item.param2 > 0);
        waypoint->setAndCheckParameter(mavlink_mission::radius.id, item.param2);
        // NOTE: Heading Required, Xtrack Location
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TO_ALT;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = 0;
        auto radius = waypoint->parameter(mavlink_mission::radius.id).toReal();
        item.param2 = waypoint->parameter(mavlink_mission::clockwise.id).toBool() ? radius
                                                                                  : -radius;
        item.param3 = 0;
        item.param4 = 0;
    }
};
} // namespace md::domain

MavlinkItemConvertorsPool::MavlinkItemConvertorsPool() :
    m_convertors({
        { mavlink_mission::home.id, new HomeConvertor() },
        { mavlink_mission::waypoint.id, new WaypointConvertor() },
        { mavlink_mission::takeoff.id, new TakeoffConvertor() },
        { mavlink_mission::landing.id, new LandingConvertor() },
        { mavlink_mission::loiterTurns.id, new LoiterTurnsConvertor() },
        { mavlink_mission::loiterAlt.id, new LoiterAltConvertor() },
    })
{
}

MavlinkItemConvertorsPool::~MavlinkItemConvertorsPool()
{
    qDeleteAll(m_convertors);
}

IMavlinkItemConvertor* MavlinkItemConvertorsPool::convertor(const WaypointType* type)
{
    return m_convertors.value(type->id, nullptr);
}

IMavlinkItemConvertor* MavlinkItemConvertorsPool::convertor(uint16_t commandType)
{
    return m_convertors.value(::commandTypes.value(commandType, nullptr), nullptr);
}
