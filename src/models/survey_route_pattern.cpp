#include "survey_route_pattern.h"

#include <QDebug>

#include "mavlink_mission_traits.h"
#include "route_pattern_algorithm_grid.h"
#include "route_pattern_algorithm_snail.h"

using namespace md::domain;

SurveyRoutePattern::SurveyRoutePattern(QObject* parent) :
    RoutePattern(&route::surveyPattern, parent)
{
}

void SurveyRoutePattern::calculate()
{
    if (m_area.isEmpty())
        return;

    QString type = this->parameter(route::surveyType.id).toString();

    const Geodetic ref = m_area.positions().first();

    QScopedPointer<IRoutePatternAlgorithm> algorithm;
    if (type == route::grid)
        algorithm.reset(
            new RoutePatternAlgorithmGrid(m_area.nedPath(ref).positions, this->parameters()));
    else if (type == route::snail)
        algorithm.reset(
            new RoutePatternAlgorithmSnail(m_area.nedPath(ref).positions, this->parameters()));
    else
        return;

    algorithm->calculate();

    QVector<Geodetic> pathPositions;
    for (const Cartesian& cartesian : algorithm->path())
    {
        pathPositions.append(ref.offsetted(cartesian));
    }
    m_path = pathPositions;

    emit pathPositionsChanged();
}
