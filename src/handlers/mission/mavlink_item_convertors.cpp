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
    { MAV_CMD_DO_SET_HOME, route::home.id },
    { MAV_CMD_NAV_WAYPOINT, route::waypoint.id },
    { MAV_CMD_NAV_TAKEOFF, route::takeoff.id },
    { MAV_CMD_DO_LAND_START, route::landStart.id },
    { MAV_CMD_NAV_LAND, route::landing.id },
    { MAV_CMD_NAV_LOITER_TURNS, route::loiterTurns.id },
    { MAV_CMD_NAV_LOITER_TO_ALT, route::loiterAlt.id },
    { MAV_CMD_NAV_LOITER_UNLIM, route::loiterUnlim.id },
    { MAV_CMD_NAV_LOITER_TIME, route::loiterTime.id },
    { MAV_CMD_DO_SET_CAM_TRIGG_DIST, route::setTriggerDist.id },
    { MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL, route::setTriggerInt.id }
};
}

namespace md::domain
{
class PositionedConvertor : public IMavlinkItemConvertor
{
public:
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        float altitude = item.z;
        if (item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
            altitude += homeAltitude;

        waypoint->position.set(Geodetic(item.x, item.y, altitude));
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.frame = MAV_FRAME_GLOBAL;

        item.x = waypoint->position().latitude();
        item.y = waypoint->position().longitude();
        item.z = waypoint->position().altitude();
    }

    bool isWaypointItem() const override
    {
        return true;
    }
};

class HomeConvertor : public PositionedConvertor
{
public:
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::home);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->setAndCheckParameter(route::yaw.id, item.param4);
        // NOTE: current/specified position
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_HOME;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = 0;
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(route::yaw.id).toReal();
    }
};

class WaypointConvertor : public PositionedConvertor
{
public:
    /* Navigate to waypoint.
     * | Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
     * | Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)
     * | 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit,
     *   negative value for counter-clockwise orbit. Allows trajectory control.
     * | Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
     * | Latitude| Longitude| Altitude|  */
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::waypoint);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->setAndCheckParameter(route::time.id, item.param1);
        waypoint->setAndCheckParameter(route::acceptRadius.id, item.param2);
        waypoint->setAndCheckParameter(route::passRadius.id, item.param3);
        waypoint->setAndCheckParameter(route::yaw.id, item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_WAYPOINT;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::time.id).toInt();
        item.param2 = waypoint->parameter(route::acceptRadius.id).toReal();
        item.param3 = waypoint->parameter(route::passRadius.id).toReal();
        item.param4 = waypoint->parameter(route::yaw.id).toReal();
    }
};

class TakeoffConvertor : public PositionedConvertor
{
public:
    /* Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.
     * Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  */
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::takeoff);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->setAndCheckParameter(route::pitch.id, item.param1);
        waypoint->setAndCheckParameter(route::yaw.id, item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_TAKEOFF;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::pitch.id).toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(route::yaw.id).toReal();
    }
};

class LandStartConvertor : public IMavlinkItemConvertor
{
public:
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::landStart);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_LAND_START;
        item.param1 = 0;
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = 0;
    }

    bool isWaypointItem() const override
    {
        return false;
    }
};

class LandingConvertor : public PositionedConvertor
{
public:
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::landing);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->setAndCheckParameter(route::abortAltitude.id, item.param1);
        waypoint->setAndCheckParameter(route::yaw.id, item.param4);
        // TODO: Precision land mode
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LAND;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::abortAltitude.id).toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(route::yaw.id).toReal();
    }
};

class LoiterTurnsConvertor : public PositionedConvertor
{
public:
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::loiterTurns);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->setAndCheckParameter(route::turns.id, item.param1);
        waypoint->setAndCheckParameter(route::headingRequired.id, item.param2);
        waypoint->setAndCheckParameter(route::clockwise.id, item.param3 > 0);
        waypoint->setAndCheckParameter(route::radius.id, qAbs(item.param3));
        waypoint->setAndCheckParameter(route::xtrack.id, item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TURNS;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::turns.id).toInt();
        item.param2 = waypoint->parameter(route::headingRequired.id).toBool();
        auto radius = waypoint->parameter(route::radius.id).toReal();
        item.param3 = waypoint->parameter(route::clockwise.id).toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(route::xtrack.id).toReal();
    }
};

class LoiterAltConvertor : public PositionedConvertor
{
public:
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::loiterAlt);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->setAndCheckParameter(route::headingRequired.id, item.param1);
        waypoint->setAndCheckParameter(route::clockwise.id, item.param2 > 0);
        waypoint->setAndCheckParameter(route::radius.id, item.param2);
        waypoint->setAndCheckParameter(route::xtrack.id, item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TO_ALT;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::headingRequired.id).toBool();
        auto radius = waypoint->parameter(route::radius.id).toReal();
        item.param2 = waypoint->parameter(route::clockwise.id).toBool() ? radius : -radius;
        item.param3 = 0;
        item.param4 = waypoint->parameter(route::xtrack.id).toReal();
    }
};

class LoiterUnlimConvertor : public PositionedConvertor
{
public:
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::loiterUnlim);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->setAndCheckParameter(route::clockwise.id, item.param3 > 0);
        waypoint->setAndCheckParameter(route::radius.id, item.param3);
        waypoint->setAndCheckParameter(route::yaw.id, item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_UNLIM;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = 0;
        item.param2 = 0;
        auto radius = waypoint->parameter(route::radius.id).toReal();
        item.param3 = waypoint->parameter(route::clockwise.id).toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(route::yaw.id).toReal();
    }
};

class LoiterTimeConvertor : public PositionedConvertor
{
public:
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::loiterTime);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->setAndCheckParameter(route::time.id, item.param1);
        waypoint->setAndCheckParameter(route::headingRequired.id, item.param2);
        waypoint->setAndCheckParameter(route::clockwise.id, item.param3 > 0);
        waypoint->setAndCheckParameter(route::radius.id, item.param3);
        waypoint->setAndCheckParameter(route::xtrack.id, item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TIME;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::time.id).toInt();
        item.param2 = waypoint->parameter(route::headingRequired.id).toBool();
        auto radius = waypoint->parameter(route::radius.id).toReal();
        item.param3 = waypoint->parameter(route::clockwise.id).toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(route::xtrack.id).toReal();
    }
};

class SetTriggerDistConvertor : public IMavlinkItemConvertor
{
public:
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypointItem) override
    {
        waypointItem->setType(&route::setTriggerDist);
        waypointItem->setAndCheckParameter(route::distance.id, item.param1);
        waypointItem->setAndCheckParameter(route::shutter.id, item.param2);
        waypointItem->setAndCheckParameter(route::trgOnce.id, item.param3);
    }

    void fromItem(const RouteItem* waypointItem, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        item.param1 = waypointItem->parameter(route::distance.id).toReal();
        item.param2 = waypointItem->parameter(route::shutter.id).toInt();
        item.param3 = waypointItem->parameter(route::trgOnce.id).toBool();
        item.param4 = 0;
        item.x = 0;
        item.y = 0;
        item.z = 0;
    }

    bool isWaypointItem() const override
    {
        return false;
    }
};
class SetTriggerIntConvertor : public IMavlinkItemConvertor
{
public:
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypointItem) override
    {
        waypointItem->setType(&route::setTriggerInt);
        waypointItem->setAndCheckParameter(route::interval.id, item.param1);
        waypointItem->setAndCheckParameter(route::shutter.id, item.param2);
    }

    void fromItem(const RouteItem* waypointItem, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        item.param1 = waypointItem->parameter(route::interval.id).toInt();
        item.param2 = waypointItem->parameter(route::shutter.id).toInt();
        item.param3 = 0;
        item.param4 = 0;
        item.x = 0;
        item.y = 0;
        item.z = 0;
    }

    bool isWaypointItem() const override
    {
        return false;
    }
};

} // namespace md::domain

MavlinkItemConvertorsPool::MavlinkItemConvertorsPool() :
    m_itemConvertors({
        { route::waypoint.id, new WaypointConvertor() },
        { route::takeoff.id, new TakeoffConvertor() },
        { route::landStart.id, new LandStartConvertor() },
        { route::landing.id, new LandingConvertor() },
        { route::loiterTurns.id, new LoiterTurnsConvertor() },
        { route::loiterAlt.id, new LoiterAltConvertor() },
        { route::loiterUnlim.id, new LoiterUnlimConvertor() },
        { route::loiterTime.id, new LoiterTimeConvertor() },
        { route::setTriggerDist.id, new SetTriggerDistConvertor() },
        { route::setTriggerInt.id, new SetTriggerIntConvertor() },
    }),
    m_homeConvertor(new HomeConvertor())
{
}

MavlinkItemConvertorsPool::~MavlinkItemConvertorsPool()
{
    qDeleteAll(m_itemConvertors);
    delete m_homeConvertor;
}

IMavlinkItemConvertor* MavlinkItemConvertorsPool::convertor(const QString& typeId) const
{
    return m_itemConvertors.value(typeId, nullptr);
}

IMavlinkItemConvertor* MavlinkItemConvertorsPool::convertor(uint16_t commandType) const
{
    return m_itemConvertors.value(::commandTypes.value(commandType, nullptr), nullptr);
}

IMavlinkItemConvertor* MavlinkItemConvertorsPool::homeConvertor() const
{
    return m_homeConvertor;
}

void MavlinkItemConvertorsPool::setHomeAltitude(float homeAltitude)
{
    for (IMavlinkItemConvertor* convertor : m_itemConvertors)
    {
        convertor->homeAltitude = homeAltitude;
    }
}
