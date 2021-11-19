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

    virtual void toItem(const mavlink_mission_item_t& item, WaypointItem* waypointItem) = 0;
    virtual void fromItem(const WaypointItem* waypointItem, mavlink_mission_item_t& item) = 0;

    virtual bool isWaypointItem() const = 0;
};

class MavlinkItemConvertorsPool
{
public:
    MavlinkItemConvertorsPool();
    ~MavlinkItemConvertorsPool();

    IMavlinkItemConvertor* convertor(const QString& typeId) const;
    IMavlinkItemConvertor* convertor(uint16_t commandType) const;
    IMavlinkItemConvertor* homeConvertor() const;

private:
    const QMap<QString, IMavlinkItemConvertor*> m_itemConvertors;
    IMavlinkItemConvertor* const m_homeConvertor;
};

} // namespace md::domain

#endif // MAVLINK_ITEM_CONVERTORS_H
