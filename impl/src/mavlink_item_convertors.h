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

    virtual void itemToWaypoint(const mavlink_mission_item_t& item, Waypoint* waypoint) = 0;
    virtual void waypointToItem(const Waypoint* waypoint, mavlink_mission_item_t& item) = 0;
};

class MavlinkItemConvertorsPool
{
public:
    MavlinkItemConvertorsPool();
    ~MavlinkItemConvertorsPool();

    IMavlinkItemConvertor* convertor(const WaypointType* type);

private:
    const QMap<const WaypointType*, IMavlinkItemConvertor*> m_convertors;
};

} // namespace md::domain

#endif // MAVLINK_ITEM_CONVERTORS_H
