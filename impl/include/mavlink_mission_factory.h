#ifndef MAVLINK_MISSION_FACTORY_H
#define MAVLINK_MISSION_FACTORY_H

#include "i_mission_factory.h"

namespace md::domain
{
class MavlinkMissionFactory : public IMissionFactory
{
public:
    MavlinkMissionFactory();

    Mission* create() override;

private:
    const MissionType* m_type;
};
} // namespace md::domain

#endif // MAVLINK_MISSION_FACTORY_H
