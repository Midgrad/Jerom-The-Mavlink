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
    if (m_areaPositions.isEmpty())
        return;

    const int spacing = this->parameter(route::spacing.id).toInt();
    const float altitude = this->parameter(route::altitude.id).toFloat();
    const float heading = this->parameter(route::heading.id).toFloat();

    const Geodetic& ref = m_areaPositions.first();

    QVector<Cartesian> area;
    for (const Geodetic& pos : qAsConst(m_areaPositions))
    {
        area.append(pos.nedPoint(ref));
    }

    double minX = area.first().x;
    double maxX = area.first().x;
    double minY = area.first().y;
    double maxY = area.first().y;
    for (const Cartesian& pos : area)
    {
        minX = qMin(minX, pos.x());
        maxX = qMax(maxX, pos.x());
        minY = qMin(minY, pos.y());
        maxY = qMax(maxY, pos.y());
    }

    m_pathPositions.clear();
    double x = minX;
    for (int i = 0;; ++i)
    {
        bool reverse = i % 2;

        m_pathPositions.append(
            ref.offsetted(Cartesian(x, reverse ? minY : maxY, -altitude).rotated(heading)));
        m_pathPositions.append(
            ref.offsetted(Cartesian(x, reverse ? maxY : minY, -altitude).rotated(heading)));

        x += spacing;
        if (x >= maxX)
            break;
    }

    emit pathPositionsChanged();
}
