#include "mavlink_mission_pattern_factory.h"

#include "mavlink_mission_traits.h"
#include "mission_pattern_algorithm_grid.h"
#include "mission_pattern_algorithm_snail.h"
#include "mavlink_mission_pattern.h"

using namespace md::domain;

RoutePattern* MavlinkMissionPatternFactory::create(const QString& routePatternId)
{
    if (routePatternId == mission::surveyPatternGrid.id)
        return new MavlinkMissionPattern(&mission::surveyPatternGrid, new RoutePatternAlgorithmGrid());
    else if (routePatternId == mission::surveyPatternSnail.id)
        return new MavlinkMissionPattern(&mission::surveyPatternSnail, new RoutePatternAlgorithmSnail());

    return nullptr;
}
