#ifndef MAVLINK_MISSION_PATTERN_FACTORY_H
#define MAVLINK_MISSION_PATTERN_FACTORY_H

#include "mission_pattern.h"

namespace md::domain
{
class MavlinkMissionPatternFactory : public IRoutePatternFactory
{
public:
    RoutePattern* create(const QString& routePatternId) override;
};
} // namespace md::domain

#endif // MAVLINK_MISSION_PATTERN_FACTORY_H
