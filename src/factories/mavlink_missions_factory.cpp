#include "mavlink_missions_factory.h"

#include "mavlink_mission_traits.h"

namespace md::domain
{
class HomeItemType : public MissionRouteItemType
{
public:
    HomeItemType() : MissionRouteItemType(mission::mavlinkHome, TR_ROUTE("Home"))
    {
    }

    ParameterMap createParameters()
    {
        ParameterMap map;
        return map;
    }
};

} // namespace md::domain

using namespace md::domain;

//constexpr char mavlinkWaypoint[] = "mavlink_waypoint";
//constexpr char mavlinkHome[] = "mavlink_home";

//constexpr char mavlinkTakeoff[] = "mavlink_takeoff";
//constexpr char mavlinkLanding[] = "mavlink_landing";
//constexpr char mavlinkLoiterTurns[] = "mavlink_loiterTurns";
//constexpr char mavlinkLoiterAlt[] = "mavlink_loiterAlt";
//constexpr char mavlinkLoiterUnlim[] = "mavlink_loiterUnlim";
//constexpr char mavlinkLoiterTime[] = "mavlink_loiterTime";

//constexpr char mavlinkSetTriggerDist[] = "mavlink_setTriggerDist";
//constexpr char mavlinkSetTriggerInt[] = "mavlink_setTriggerInt";

//constexpr char mavlinkLandStart[] = "mavlink_land_start";

MavlinkMissionsFactory::MavlinkMissionsFactory() :
    m_itemTypes({ { mission::mavlinkHome, new HomeItemType() } })
{
}

Mission* MavlinkMissionsFactory::createMission(const QVariantMap& map)
{
    return new Mission(map.value(props::type).toString(), map.value(props::name).toString(),
                       map.value(props::vehicle), map.value(props::id));
}

MissionRouteItem* MavlinkMissionsFactory::createMissionRouteItem(const QVariantMap& map)
{
    auto type = m_itemTypes.value(map.value(props::type).toString());
    if (!type)
        return nullptr;

    return new MissionRouteItem(type);
}
