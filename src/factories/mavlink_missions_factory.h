#ifndef MAVLINK_MISSIONS_FACTORY_H
#define MAVLINK_MISSIONS_FACTORY_H

#include "i_missions_factory.h"

namespace md::domain
{
class MavlinkMissionsFactory : public IMissionsFactory
{
public:
    MavlinkMissionsFactory();
    ~MavlinkMissionsFactory() override;

    Mission* createMission(const QVariantMap& map) override;
    MissionRouteItem* createMissionRouteItem(const QVariantMap& map) override;

private:
    QMap<QString, MissionRouteItemType*> m_itemTypes;
};
} // namespace md::domain

#endif // MAVLINK_MISSIONS_FACTORY_H
