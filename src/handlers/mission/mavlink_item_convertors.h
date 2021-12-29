#ifndef MAVLINK_ITEM_CONVERTORS_H
#define MAVLINK_ITEM_CONVERTORS_H

#include <common/mavlink.h>

#include "route_item.h"

namespace md::domain
{
struct MavlinkConvertorParams
{
    float homeAltitude = 0;
    double lastX = 0;
    double lastY = 0;
};

class IMavlinkItemConvertor
{
public:
    IMavlinkItemConvertor(MavlinkConvertorParams* params) : m_params(params)
    {
    }
    virtual ~IMavlinkItemConvertor() = default;

    virtual void toItem(const mavlink_mission_item_t& item, RouteItem* waypointItem) = 0;
    virtual void fromItem(const RouteItem* waypointItem, mavlink_mission_item_t& item) = 0;

    virtual bool isWaypointItem() const = 0;

protected:
    MavlinkConvertorParams* m_params;
};

class MavlinkItemConvertorsPool
{
public:
    MavlinkItemConvertorsPool();
    ~MavlinkItemConvertorsPool();

    IMavlinkItemConvertor* convertor(const QString& typeId) const;
    IMavlinkItemConvertor* convertor(uint16_t commandType) const;
    IMavlinkItemConvertor* homeConvertor() const;

    void setHomeAltitude(float homeAltitude);

private:
    MavlinkConvertorParams m_params;
    const QMap<QString, IMavlinkItemConvertor*> m_itemConvertors;
    IMavlinkItemConvertor* const m_homeConvertor;
};

} // namespace md::domain

#endif // MAVLINK_ITEM_CONVERTORS_H
