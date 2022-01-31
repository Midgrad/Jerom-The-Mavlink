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
//MAV_CMD_NAV_VTOL_TAKEOFF

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
    explicit PositionedConvertor(MavlinkConvertorParams* params) : IMavlinkItemConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        float altitude = item.z;
        if (item.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
            altitude += m_params->homeAltitude;

        double x = item.x;
        double y = item.y;
        if (qFuzzyIsNull(x) && qFuzzyIsNull(y))
        {
            x = m_params->lastX;
            y = m_params->lastY;

            if (qFuzzyIsNull(item.z))
                altitude = m_params->lastAltitude;
        }
        else
        {
            m_params->lastX = x;
            m_params->lastY = y;
            m_params->lastAltitude = altitude;
        }

        waypoint->position.set(Geodetic(x, y, altitude));
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
    explicit HomeConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::home);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(route::yaw.id)->setValue(item.param4);
        // NOTE: current/specified position
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_HOME;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = 0;
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(route::yaw.id)->value().toReal();
    }
};

class WaypointConvertor : public PositionedConvertor
{
public:
    explicit WaypointConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

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
        waypoint->parameter(route::time.id)->setValue(item.param1);
        waypoint->parameter(route::acceptRadius.id)->setValue(item.param2);
        waypoint->parameter(route::passRadius.id)->setValue(item.param3);
        waypoint->parameter(route::yaw.id)->setValue(item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_WAYPOINT;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::time.id)->value().toInt();
        item.param2 = waypoint->parameter(route::acceptRadius.id)->value().toReal();
        item.param3 = waypoint->parameter(route::passRadius.id)->value().toReal();
        item.param4 = utils::guardNaN(waypoint->parameter(route::yaw.id)->value());
    }
};

class TakeoffConvertor : public PositionedConvertor
{
public:
    explicit TakeoffConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    /* Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.
     * Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  */
    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::takeoff);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(route::pitch.id)->setValue(item.param1);
        waypoint->parameter(route::yaw.id)->setValue(item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_TAKEOFF;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::pitch.id)->value().toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(route::yaw.id)->value().toReal();
    }
};

class LandStartConvertor : public PositionedConvertor
{
public:
    explicit LandStartConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::landStart);
        PositionedConvertor::toItem(item, waypoint);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_LAND_START;
        PositionedConvertor::fromItem(waypoint, item);
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
    explicit LandingConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::landing);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(route::abortAltitude.id)->setValue(item.param1);
        waypoint->parameter(route::yaw.id)->setValue(item.param4);
        // TODO: Precision land mode
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LAND;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::abortAltitude.id)->value().toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(route::yaw.id)->value().toReal();
    }
};

class LoiterTurnsConvertor : public PositionedConvertor
{
public:
    explicit LoiterTurnsConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::loiterTurns);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(route::turns.id)->setValue(item.param1);
        waypoint->parameter(route::headingRequired.id)->setValue(item.param2);
        waypoint->parameter(route::clockwise.id)->setValue(item.param3 > 0);
        waypoint->parameter(route::radius.id)->setValue(qAbs(item.param3));
        waypoint->parameter(route::xtrack.id)->setValue(item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TURNS;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::turns.id)->value().toInt();
        item.param2 = waypoint->parameter(route::headingRequired.id)->value().toBool();
        auto radius = waypoint->parameter(route::radius.id)->value().toReal();
        item.param3 = waypoint->parameter(route::clockwise.id)->value().toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(route::xtrack.id)->value().toReal();
    }
};

class LoiterAltConvertor : public PositionedConvertor
{
public:
    explicit LoiterAltConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::loiterAlt);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(route::headingRequired.id)->setValue(item.param1);
        waypoint->parameter(route::clockwise.id)->setValue(item.param2 > 0);
        waypoint->parameter(route::radius.id)->setValue(item.param2);
        waypoint->parameter(route::xtrack.id)->setValue(item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TO_ALT;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::headingRequired.id)->value().toBool();
        auto radius = waypoint->parameter(route::radius.id)->value().toReal();
        item.param2 = waypoint->parameter(route::clockwise.id)->value().toBool() ? radius : -radius;
        item.param3 = 0;
        item.param4 = waypoint->parameter(route::xtrack.id)->value().toReal();
    }
};

class LoiterUnlimConvertor : public PositionedConvertor
{
public:
    explicit LoiterUnlimConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::loiterUnlim);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(route::clockwise.id)->setValue(item.param3 > 0);
        waypoint->parameter(route::radius.id)->setValue(item.param3);
        waypoint->parameter(route::yaw.id)->setValue(item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_UNLIM;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = 0;
        item.param2 = 0;
        auto radius = waypoint->parameter(route::radius.id)->value().toReal();
        item.param3 = waypoint->parameter(route::clockwise.id)->value().toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(route::yaw.id)->value().toReal();
    }
};

class LoiterTimeConvertor : public PositionedConvertor
{
public:
    explicit LoiterTimeConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, RouteItem* waypoint) override
    {
        waypoint->setType(&route::loiterTime);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(route::time.id)->setValue(item.param1);
        waypoint->parameter(route::headingRequired.id)->setValue(item.param2);
        waypoint->parameter(route::clockwise.id)->setValue(item.param3 > 0);
        waypoint->parameter(route::radius.id)->setValue(item.param3);
        waypoint->parameter(route::xtrack.id)->setValue(item.param4);
    }

    void fromItem(const RouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TIME;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(route::time.id)->value().toInt();
        item.param2 = waypoint->parameter(route::headingRequired.id)->value().toBool();
        auto radius = waypoint->parameter(route::radius.id)->value().toReal();
        item.param3 = waypoint->parameter(route::clockwise.id)->value().toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(route::xtrack.id)->value().toReal();
    }
};

class SetTriggerDistConvertor : public IMavlinkItemConvertor
{
public:
    explicit SetTriggerDistConvertor(MavlinkConvertorParams* params) : IMavlinkItemConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, RouteItem* waypointItem) override
    {
        waypointItem->setType(&route::setTriggerDist);
        waypointItem->parameter(route::distance.id)->setValue(item.param1);
        waypointItem->parameter(route::shutter.id)->setValue(item.param2);
        waypointItem->parameter(route::trgOnce.id)->setValue(item.param3);
    }

    void fromItem(const RouteItem* waypointItem, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        item.param1 = waypointItem->parameter(route::distance.id)->value().toReal();
        item.param2 = waypointItem->parameter(route::shutter.id)->value().toInt();
        item.param3 = waypointItem->parameter(route::trgOnce.id)->value().toBool();
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
    explicit SetTriggerIntConvertor(MavlinkConvertorParams* params) : IMavlinkItemConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, RouteItem* waypointItem) override
    {
        waypointItem->setType(&route::setTriggerInt);
        waypointItem->parameter(route::interval.id)->setValue(item.param1);
        waypointItem->parameter(route::shutter.id)->setValue(item.param2);
    }

    void fromItem(const RouteItem* waypointItem, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        item.param1 = waypointItem->parameter(route::interval.id)->value().toInt();
        item.param2 = waypointItem->parameter(route::shutter.id)->value().toInt();
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
        { route::waypoint.id, new WaypointConvertor(&m_params) },
        { route::takeoff.id, new TakeoffConvertor(&m_params) },
        { route::landStart.id, new LandStartConvertor(&m_params) },
        { route::landing.id, new LandingConvertor(&m_params) },
        { route::loiterTurns.id, new LoiterTurnsConvertor(&m_params) },
        { route::loiterAlt.id, new LoiterAltConvertor(&m_params) },
        { route::loiterUnlim.id, new LoiterUnlimConvertor(&m_params) },
        { route::loiterTime.id, new LoiterTimeConvertor(&m_params) },
        { route::setTriggerDist.id, new SetTriggerDistConvertor(&m_params) },
        { route::setTriggerInt.id, new SetTriggerIntConvertor(&m_params) },
    }),
    m_homeConvertor(new HomeConvertor(&m_params))
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
    m_params.homeAltitude = homeAltitude;
}
