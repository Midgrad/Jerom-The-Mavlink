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
    explicit PositionedConvertor(MavlinkConvertorParams* params) : IMavlinkItemConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypoint) override
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

    void fromItem(const MissionRouteItem* waypoint, mavlink_mission_item_t& item) override
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

    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypoint) override
    {
        waypoint->setType(&mission::home);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(mission::yaw.id)->setValue(item.param4);
        // NOTE: current/specified position
    }

    void fromItem(const MissionRouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_HOME;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = 0;
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mission::yaw.id)->value().toReal();
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
    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypoint) override
    {
        waypoint->setType(&mission::waypoint);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(mission::time.id)->setValue(item.param1);
        waypoint->parameter(mission::acceptRadius.id)->setValue(item.param2);
        waypoint->parameter(mission::passRadius.id)->setValue(item.param3);
        waypoint->parameter(mission::yaw.id)->setValue(item.param4);
    }

    void fromItem(const MissionRouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_WAYPOINT;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(mission::time.id)->value().toInt();
        item.param2 = waypoint->parameter(mission::acceptRadius.id)->value().toReal();
        item.param3 = waypoint->parameter(mission::passRadius.id)->value().toReal();
        item.param4 = utils::guardNaN(waypoint->parameter(mission::yaw.id)->value());
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
    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypoint) override
    {
        waypoint->setType(&mission::takeoff);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(mission::pitch.id)->setValue(item.param1);
        waypoint->parameter(mission::yaw.id)->setValue(item.param4);
    }

    void fromItem(const MissionRouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_TAKEOFF;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(mission::pitch.id)->value().toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mission::yaw.id)->value().toReal();
    }
};

class LandStartConvertor : public PositionedConvertor
{
public:
    explicit LandStartConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypoint) override
    {
        waypoint->setType(&mission::landStart);
        PositionedConvertor::toItem(item, waypoint);
    }

    void fromItem(const MissionRouteItem* waypoint, mavlink_mission_item_t& item) override
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

    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypoint) override
    {
        waypoint->setType(&mission::landing);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(mission::abortAltitude.id)->setValue(item.param1);
        waypoint->parameter(mission::yaw.id)->setValue(item.param4);
        // TODO: Precision land mode
    }

    void fromItem(const MissionRouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LAND;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(mission::abortAltitude.id)->value().toReal();
        item.param2 = 0;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mission::yaw.id)->value().toReal();
    }
};

class LoiterTurnsConvertor : public PositionedConvertor
{
public:
    explicit LoiterTurnsConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypoint) override
    {
        waypoint->setType(&mission::loiterTurns);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(mission::turns.id)->setValue(item.param1);
        waypoint->parameter(mission::headingRequired.id)->setValue(item.param2);
        waypoint->parameter(mission::clockwise.id)->setValue(item.param3 > 0);
        waypoint->parameter(mission::radius.id)->setValue(qAbs(item.param3));
        waypoint->parameter(mission::xtrack.id)->setValue(item.param4);
    }

    void fromItem(const MissionRouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TURNS;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(mission::turns.id)->value().toInt();
        item.param2 = waypoint->parameter(mission::headingRequired.id)->value().toBool();
        auto radius = waypoint->parameter(mission::radius.id)->value().toReal();
        item.param3 = waypoint->parameter(mission::clockwise.id)->value().toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(mission::xtrack.id)->value().toReal();
    }
};

class LoiterAltConvertor : public PositionedConvertor
{
public:
    explicit LoiterAltConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypoint) override
    {
        waypoint->setType(&mission::loiterAlt);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(mission::headingRequired.id)->setValue(item.param1);
        waypoint->parameter(mission::clockwise.id)->setValue(item.param2 > 0);
        waypoint->parameter(mission::radius.id)->setValue(item.param2);
        waypoint->parameter(mission::xtrack.id)->setValue(item.param4);
    }

    void fromItem(const MissionRouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TO_ALT;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(mission::headingRequired.id)->value().toBool();
        auto radius = waypoint->parameter(mission::radius.id)->value().toReal();
        item.param2 = waypoint->parameter(mission::clockwise.id)->value().toBool() ? radius : -radius;
        item.param3 = 0;
        item.param4 = waypoint->parameter(mission::xtrack.id)->value().toReal();
    }
};

class LoiterUnlimConvertor : public PositionedConvertor
{
public:
    explicit LoiterUnlimConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypoint) override
    {
        waypoint->setType(&mission::loiterUnlim);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(mission::clockwise.id)->setValue(item.param3 > 0);
        waypoint->parameter(mission::radius.id)->setValue(item.param3);
        waypoint->parameter(mission::yaw.id)->setValue(item.param4);
    }

    void fromItem(const MissionRouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_UNLIM;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = 0;
        item.param2 = 0;
        auto radius = waypoint->parameter(mission::radius.id)->value().toReal();
        item.param3 = waypoint->parameter(mission::clockwise.id)->value().toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(mission::yaw.id)->value().toReal();
    }
};

class LoiterTimeConvertor : public PositionedConvertor
{
public:
    explicit LoiterTimeConvertor(MavlinkConvertorParams* params) : PositionedConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypoint) override
    {
        waypoint->setType(&mission::loiterTime);
        PositionedConvertor::toItem(item, waypoint);
        waypoint->parameter(mission::time.id)->setValue(item.param1);
        waypoint->parameter(mission::headingRequired.id)->setValue(item.param2);
        waypoint->parameter(mission::clockwise.id)->setValue(item.param3 > 0);
        waypoint->parameter(mission::radius.id)->setValue(item.param3);
        waypoint->parameter(mission::xtrack.id)->setValue(item.param4);
    }

    void fromItem(const MissionRouteItem* waypoint, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_NAV_LOITER_TIME;
        PositionedConvertor::fromItem(waypoint, item);
        item.param1 = waypoint->parameter(mission::time.id)->value().toInt();
        item.param2 = waypoint->parameter(mission::headingRequired.id)->value().toBool();
        auto radius = waypoint->parameter(mission::radius.id)->value().toReal();
        item.param3 = waypoint->parameter(mission::clockwise.id)->value().toBool() ? radius : -radius;
        item.param4 = waypoint->parameter(mission::xtrack.id)->value().toReal();
    }
};

class SetTriggerDistConvertor : public IMavlinkItemConvertor
{
public:
    explicit SetTriggerDistConvertor(MavlinkConvertorParams* params) : IMavlinkItemConvertor(params)
    {
    }

    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypointItem) override
    {
        waypointItem->setType(&mission::setTriggerDist);
        waypointItem->parameter(mission::distance.id)->setValue(item.param1);
        waypointItem->parameter(mission::shutter.id)->setValue(item.param2);
        waypointItem->parameter(mission::trgOnce.id)->setValue(item.param3);
    }

    void fromItem(const MissionRouteItem* waypointItem, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_CAM_TRIGG_DIST;
        item.param1 = waypointItem->parameter(mission::distance.id)->value().toReal();
        item.param2 = waypointItem->parameter(mission::shutter.id)->value().toInt();
        item.param3 = waypointItem->parameter(mission::trgOnce.id)->value().toBool();
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

    void toItem(const mavlink_mission_item_t& item, MissionRouteItem* waypointItem) override
    {
        waypointItem->setType(&mission::setTriggerInt);
        waypointItem->parameter(mission::interval.id)->setValue(item.param1);
        waypointItem->parameter(mission::shutter.id)->setValue(item.param2);
    }

    void fromItem(const MissionRouteItem* waypointItem, mavlink_mission_item_t& item) override
    {
        item.command = MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL;
        item.param1 = waypointItem->parameter(mission::interval.id)->value().toInt();
        item.param2 = waypointItem->parameter(mission::shutter.id)->value().toInt();
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
        { mission::waypoint.id, new WaypointConvertor(&m_params) },
        { mission::takeoff.id, new TakeoffConvertor(&m_params) },
        { mission::landStart.id, new LandStartConvertor(&m_params) },
        { mission::landing.id, new LandingConvertor(&m_params) },
        { mission::loiterTurns.id, new LoiterTurnsConvertor(&m_params) },
        { mission::loiterAlt.id, new LoiterAltConvertor(&m_params) },
        { mission::loiterUnlim.id, new LoiterUnlimConvertor(&m_params) },
        { mission::loiterTime.id, new LoiterTimeConvertor(&m_params) },
        { mission::setTriggerDist.id, new SetTriggerDistConvertor(&m_params) },
        { mission::setTriggerInt.id, new SetTriggerIntConvertor(&m_params) },
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
