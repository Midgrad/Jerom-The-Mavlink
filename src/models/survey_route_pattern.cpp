#include "survey_route_pattern.h"

#include <QDebug>

#include "mavlink_mission_traits.h"

using namespace md::domain;

SurveyRoutePattern::SurveyRoutePattern(const RoutePatternType* type,
                                       IRoutePatternAlgorithm* algorithm, QObject* parent) :
    RoutePattern(type, parent),
    m_algorithm(algorithm)
{
}

SurveyRoutePattern::~SurveyRoutePattern()
{
}

void SurveyRoutePattern::calculate()
{
    if (m_area.isEmpty())
        return;

    const Geodetic ref = m_area.positions().first();

    auto cartesians = m_algorithm->calculate(m_area.nedPath(ref).positions, this->parameters());

    QVector<Geodetic> pathPositions;
    for (const Cartesian& cartesian : cartesians)
    {
        pathPositions.append(ref.offsetted(cartesian));
    }
    m_path = pathPositions;

    emit pathPositionsChanged();
}
