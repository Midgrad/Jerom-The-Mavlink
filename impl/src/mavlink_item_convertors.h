#ifndef MAVLINK_ITEM_CONVERTORS_H
#define MAVLINK_ITEM_CONVERTORS_H

#include <common/mavlink.h>

#include "waypoint.h"

namespace md::domain
{
class IMavlinkItemConvertor
{
public:
    virtual ~IMavlinkItemConvertor() = default;

    virtual void toItem(const mavlink_mission_item_t& item, RouteItem* routeItem) = 0;
    virtual void fromItem(const RouteItem* routeItem, mavlink_mission_item_t& item) = 0;
};

class IMavlinkWaypointConvertor
{
public:
    virtual ~IMavlinkWaypointConvertor() = default;

    virtual void toWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) = 0;
    virtual void fromWaypoint(const Waypoint* waypoint, mavlink_mission_item_t& item) = 0;
};

class MavlinkItemConvertorsPool
{
public:
    MavlinkItemConvertorsPool();
    ~MavlinkItemConvertorsPool();

    IMavlinkWaypointConvertor* convertor(const QString& typeId);
    IMavlinkWaypointConvertor* convertor(uint16_t commandType);

private:
    const QMap<QString, IMavlinkItemConvertor*> m_itemConvertors;
    const QMap<QString, IMavlinkWaypointConvertor*> m_waypointConvertors;
};

} // namespace md::domain

#endif // MAVLINK_ITEM_CONVERTORS_H
