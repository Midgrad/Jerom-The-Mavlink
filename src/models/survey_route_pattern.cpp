#include "survey_route_pattern.h"

#include <QDebug>

#include "mavlink_mission_traits.h"
#include "route_pattern_algorithm_grid.h"

using namespace md::domain;

SurveyRoutePattern::SurveyRoutePattern(QObject* parent) :
    RoutePattern(&route::surveyPattern, parent)
{
}

void SurveyRoutePattern::calculate()
{
    if (m_area.isEmpty())
        return;

    const Geodetic ref = m_area.positions().first();
    RoutePatternAlgorithmGrid grid(m_area.nedPath(ref).positions, this->parameters());
    grid.calculate();

    QVector<Geodetic> pathPositions;
    for (const Cartesian& cartesian : grid.path().positions())
    {
        pathPositions.append(ref.offsetted(cartesian));
    }
    m_path = pathPositions;

    emit pathPositionsChanged();
}
