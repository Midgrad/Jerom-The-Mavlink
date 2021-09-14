#ifndef MAVLINK_MISSION_FACTORY_H
#define MAVLINK_MISSION_FACTORY_H

#include "i_mission_factory.h"

namespace md::domain
{
// TODO: to mavlink traits
static const QString mavlinkType = "Mavlink mission";

class MavlinkMissionFactory : public IMissionFactory
{
public:
    MavlinkMissionFactory();

    Mission* create() override;
};
} // namespace md::domain

#endif // MAVLINK_MISSION_FACTORY_H
