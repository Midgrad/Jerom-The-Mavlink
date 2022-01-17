#include "survey_route_pattern.h"

#include <QDebug>

#include "mavlink_mission_traits.h"

using namespace md::domain;

SurveyRoutePattern::SurveyRoutePattern(QObject* parent) :
    RoutePattern(&route::surveyPattern, parent)
{
}

void SurveyRoutePattern::calculate()
{
    if (m_area.isEmpty())
        return;

    const int spacing = this->parameter(route::spacing.id).toInt();
    const float altitude = this->parameter(route::altitude.id).toFloat();
    const float heading = this->parameter(route::heading.id).toFloat();

    Geodetic ref = m_area.positions().first();

    CartesianPath area = m_area.nedPath(ref);
    double minX = area.positions().first().x;
    double maxX = area.positions().first().x;
    double minY = area.positions().first().y;
    double maxY = area.positions().first().y;
    for (const Cartesian& pos : area.positions())
    {
        minX = qMin(minX, pos.x());
        maxX = qMax(maxX, pos.x());
        minY = qMin(minY, pos.y());
        maxY = qMax(maxY, pos.y());
    }

    QVector<Geodetic> pathPositions;
    double x = minX;
    for (int i = 0;; ++i)
    {
        bool reverse = i % 2;

        pathPositions.append(
            ref.offsetted(Cartesian(x, reverse ? minY : maxY, -altitude).rotated(heading)));
        pathPositions.append(
            ref.offsetted(Cartesian(x, reverse ? maxY : minY, -altitude).rotated(heading)));

        x += spacing;
        if (x >= maxX)
            break;
    }

    m_path = pathPositions;
    emit pathPositionsChanged();
}
