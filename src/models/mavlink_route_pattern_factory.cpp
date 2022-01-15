#include "mavlink_route_pattern_factory.h"

#include "mavlink_mission_traits.h"
#include "survey_route_pattern.h"

using namespace md::domain;

RoutePattern* MavlinkRoutePatternFactory::create(const QString& routePatternId)
{
    if (routePatternId == route::surveyPattern.id)
        return new SurveyRoutePattern();

    return nullptr;
}
