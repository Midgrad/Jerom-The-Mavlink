#include "mavlink_item_convertors.h"

#include <QDebug>

#include "mavlink_mission_traits.h"
#include "mavlink_protocol_helpers.h"

using namespace md::domain;

// TODO:
//MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT
//MAV_CMD_DO_CHANGE_SPEED
//MAV_CMD_DO_CHANGE_ALTITUDE
//MAV_CMD_DO_SET_RELAY
//MAV_CMD_DO_REPEAT_RELAY
//MAV_CMD_DO_SET_SERVO
//MAV_CMD_DO_REPEAT_SERVO

namespace
{
const QMap<uint16_t, QString> commandTypes = {
    { MAV_CMD_DO_SET_HOME, mission::home.id },
    { MAV_CMD_NAV_WAYPOINT, mission::waypoint.id },
    { MAV_CMD_NAV_TAKEOFF, mission::takeoff.id },
    { MAV_CMD_DO_LAND_START, mission::landStart.id },
    { MAV_CMD_NAV_LAND, mission::landing.id },
    { MAV_CMD_NAV_LOITER_TURNS, mission::loiterTurns.id },
    { MAV_CMD_NAV_LOITER_TO_ALT, mission::loiterAlt.id },
    { MAV_CMD_NAV_LOITER_UNLIM, mission::loiterUnlim.id },
    { MAV_CMD_NAV_LOITER_TIME, mission::loiterTime.id },
    { MAV_CMD_DO_SET_CAM_TRIGG_DIST, mission::setTriggerDist.id },
    { MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL, mission::setTriggerInt.id }
};
}

namespace md::domain
{
class PositionedConvertor : public IMavlinkItemConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setAndCheckParameter(mission::relativeAlt.id,
                                       item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT);
        waypoint->setAndCheckParameter(mission::latitude.id, item.x);
        waypoint->setAndCheckParameter(mission::longitude.id, item.y);
        waypoint->setAndCheckParameter(mission::altitude.id, item.z);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.frame = waypoint->parameter(mission::relativeAlt.id).toBool()
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
        waypoint->setType(&mission::home);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mission::yaw.id, item.param4);
        // NOTE: current/specified position
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_HOME;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = 0;
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mission::yaw.id).toReal();
    }
};

class WaypointConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mission::waypoint);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mission::time.id, item.param1);
        waypoint->setAndCheckParameter(mission::radius.id, item.param2);
        waypoint->setAndCheckParameter(mission::passRadius.id, item.param3);
        waypoint->setAndCheckParameter(mission::yaw.id, item.param4);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_WAYPOINT;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mission::time.id).toInt();
        item.param2 = waypoint->parameter(mission::radius.id).toReal();
        item.param3 = waypoint->parameter(mission::passRadius.id).toReal();
        item.param4 = waypoint->parameter(mission::yaw.id).toReal();
    }
};

class TakeoffConvertor : public IMavlinkItemConvertor
{
public:
    /* Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.
     * Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  */
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mission::takeoff);
        waypoint->setAndCheckParameter(mission::relativeAlt.id,
                                       item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT);
        waypoint->setAndCheckParameter(mission::altitude.id, item.z);
        waypoint->setAndCheckParameter(mission::pitch.id, item.param1);
        waypoint->setAndCheckParameter(mission::yaw.id, item.param4);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_TAKEOFF;
        item.frame = waypoint->parameter(mission::relativeAlt.id).toBool()
                         ? MAV_FRAME_GLOBAL_RELATIVE_ALT
                         : MAV_FRAME_GLOBAL;
        item.z = waypoint->parameter(mission::altitude.id).toReal();
        item.param1 = waypoint->parameter(mission::pitch.id).toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mission::yaw.id).toReal();
    }
};

class LandStartConvertor : public IMavlinkItemConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mission::landStart);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_LAND_START;
        item.param1 = 0;
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = 0;
    }
};

class LandingConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mission::landing);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mission::abortAltitude.id, item.param1);
        waypoint->setAndCheckParameter(mission::yaw.id, item.param4);
        // NOTE: Precision land mode
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LAND;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mission::abortAltitude.id).toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mission::yaw.id).toReal();
    }
};

class LoiterTurnsConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mission::loiterTurns);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mission::turns.id, item.param1);
        waypoint->setAndCheckParameter(mission::headingRequired.id, item.param2);
        waypoint->setAndCheckParameter(mission::clockwise.id, item.param3 > 0);
        waypoint->setAndCheckParameter(mission::radius.id, item.param3);
        waypoint->setAndCheckParameter(mission::xtrack.id, item.param4);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TURNS;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mission::turns.id).toInt();
        item.param2 = waypoint->parameter(mission::headingRequired.id).toBool();
        auto radius = waypoint->parameter(mission::radius.id).toReal();
        item.param3 = waypoint->parameter(mission::clockwise.id).toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(mission::xtrack.id).toReal();
    }
};

class LoiterAltConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mission::loiterAlt);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mission::headingRequired.id, item.param1);
        waypoint->setAndCheckParameter(mission::clockwise.id, item.param2 > 0);
        waypoint->setAndCheckParameter(mission::radius.id, item.param2);
        waypoint->setAndCheckParameter(mission::xtrack.id, item.param4);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TO_ALT;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mission::headingRequired.id).toBool();
        auto radius = waypoint->parameter(mission::radius.id).toReal();
        item.param2 = waypoint->parameter(mission::clockwise.id).toBool() ? radius : -radius;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mission::xtrack.id).toReal();
    }
};

class LoiterUnlimConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mission::loiterUnlim);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mission::clockwise.id, item.param3 > 0);
        waypoint->setAndCheckParameter(mission::radius.id, item.param3);
        waypoint->setAndCheckParameter(mission::yaw.id, item.param4);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_UNLIM;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = 0;
        item.param2 = 0;
        auto radius = waypoint->parameter(mission::radius.id).toReal();
        item.param3 = waypoint->parameter(mission::clockwise.id).toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(mission::yaw.id).toReal();
    }
};

class LoiterTimeConvertor : public PositionedConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mission::loiterTime);
        PositionedConvertor::itemToWaypoint(item, waypoint);
        waypoint->setAndCheckParameter(mission::time.id, item.param1);
        waypoint->setAndCheckParameter(mission::headingRequired.id, item.param2);
        waypoint->setAndCheckParameter(mission::clockwise.id, item.param3 > 0);
        waypoint->setAndCheckParameter(mission::radius.id, item.param3);
        waypoint->setAndCheckParameter(mission::xtrack.id, item.param4);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TIME;
        PositionedConvertor::waypointToItem(waypoint, item);
        item.param1 = waypoint->parameter(mission::time.id).toInt();
        item.param2 = waypoint->parameter(mission::headingRequired.id).toBool();
        auto radius = waypoint->parameter(mission::radius.id).toReal();
        item.param3 = waypoint->parameter(mission::clockwise.id).toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(mission::xtrack.id).toReal();
    }
};

class SetTriggerDistConvertor : public IMavlinkItemConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mission::setTriggerDist);
        waypoint->setAndCheckParameter(mission::distance.id, item.param1);
        waypoint->setAndCheckParameter(mission::shutter.id, item.param2);
        waypoint->setAndCheckParameter(mission::trgOnce.id, item.param3);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        item.param1 = waypoint->parameter(mission::distance.id).toReal();
        item.param2 = waypoint->parameter(mission::shutter.id).toInt();
        item.param3 = waypoint->parameter(mission::trgOnce.id).toBool();
        item.param4 = 0;
    }
};
class SetTriggerIntConvertor : public IMavlinkItemConvertor
{
public:
    void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) override
    {
        waypoint->setType(&mission::setTriggerInt);
        waypoint->setAndCheckParameter(mission::interval.id, item.param1);
        waypoint->setAndCheckParameter(mission::shutter.id, item.param2);
    }

    void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        item.param1 = waypoint->parameter(mission::interval.id).toInt();
        item.param2 = waypoint->parameter(mission::shutter.id).toInt();
        item.param3 = 0;
        item.param4 = 0;
    }
};

} // namespace md::domain

MavlinkItemConvertorsPool::MavlinkItemConvertorsPool() :
    m_convertors({
        { mission::home.id, new HomeConvertor() },
        { mission::waypoint.id, new WaypointConvertor() },
        { mission::takeoff.id, new TakeoffConvertor() },
        { mission::landStart.id, new LandStartConvertor() },
        { mission::landing.id, new LandingConvertor() },
        { mission::loiterTurns.id, new LoiterTurnsConvertor() },
        { mission::loiterAlt.id, new LoiterAltConvertor() },
        { mission::loiterUnlim.id, new LoiterUnlimConvertor() },
        { mission::loiterTime.id, new LoiterTimeConvertor() },
        { mission::setTriggerDist.id, new SetTriggerDistConvertor() },
        { mission::setTriggerInt.id, new SetTriggerIntConvertor() },
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
