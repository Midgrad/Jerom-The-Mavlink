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
    const bool doubled = this->parameter(route::doubled.id).toBool();

    const Geodetic ref = m_area.positions().first();
    const CartesianPath areaNed = m_area.nedPath(ref);
    const CartesianRect boundingRect = areaNed.boundingRect();
    const Cartesian center = boundingRect.center();

    // Take the rect guaranteed covered survey area
    const double minX = center.x() - boundingRect.diagonal() / 2;
    const double maxX = center.x() + boundingRect.diagonal() / 2;
    const double maxY = center.y() + boundingRect.diagonal() / 2;
    double y = center.y() - boundingRect.diagonal() / 2;

    // In: CartesianPath area, spacing, heading
    // Out: CartesianPath path

    QVector<Geodetic> pathPositions;

    for (int i = 0;; ++i)
    {
        Cartesian castPoint = Cartesian(minX, y, -altitude).rotated(heading, center);
        CartesianLine cast(castPoint, Cartesian(maxX, y, -altitude).rotated(heading, center));

        QVector<Cartesian> intersections = CartesianPath(areaNed.intersections2D(cast, true))
                                               .sortedByDistance(castPoint);

        if (intersections.count() >= 2)
        {
            if (i % 2)
                std::reverse(intersections.begin(), intersections.end());
            for (const Cartesian& point : intersections)
            {
                pathPositions.append(ref.offsetted(point));
            }
        }
        y += spacing;
        if (y >= maxY)
            break;
    }

    m_path = pathPositions;
    emit pathPositionsChanged();
}
