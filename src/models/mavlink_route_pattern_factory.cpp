#include "mavlink_route_pattern_factory.h"

#include "mavlink_mission_traits.h"
#include "route_pattern_algorithm_grid.h"
#include "route_pattern_algorithm_snail.h"
#include "survey_route_pattern.h"

using namespace md::domain;

RoutePattern* MavlinkRoutePatternFactory::create(const QString& routePatternId)
{
    if (routePatternId == route::surveyPatternGrid.id)
        return new SurveyRoutePattern(new RoutePatternAlgorithmGrid());
    else if (routePatternId == route::surveyPatternSnail.id)
        return new SurveyRoutePattern(new RoutePatternAlgorithmSnail());

    return nullptr;
}
