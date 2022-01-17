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

    const Geodetic ref = m_area.positions().first();
    const CartesianPath areaNed = m_area.nedPath(ref); // TODO .rotated(heading, center)
    const CartesianRect boundingRect = areaNed.boundingRect();
    const Cartesian center = boundingRect.center();

    // Take the rect guaranteed covered survey area
    const double minX = center.x() - boundingRect.diagonal() / 2;
    const double maxX = center.x() + boundingRect.diagonal() / 2;
    const double maxY = center.y() + boundingRect.diagonal() / 2;
    double y = center.y() - boundingRect.diagonal() / 2;

    QVector<Geodetic> pathPositions;

    for (int i = 0;; ++i)
    {
        Cartesian castPoint(minX, y, -altitude);
        CartesianLine cast(castPoint, Cartesian(maxX, y, -altitude));

        auto intersections = areaNed.intersections2D(cast, true);
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
    // WHOLE
    //    for (int i = 0;; ++i)
    //    {
    //        bool reverse = i % 2;

    //        pathPositions.append(
    //            ref.offsetted(Cartesian(reverse ? minX : maxX, y, -altitude).rotated(heading, center)));
    //        pathPositions.append(
    //            ref.offsetted(Cartesian(reverse ? maxX : minX, y, -altitude).rotated(heading, center)));

    //        y += spacing;
    //        if (y >= maxY)
    //            break;
    //    }

    // RECT
    //    pathPositions.append(ref.offsetted(boundingRect.topLeft().rotated(heading, center)));
    //    pathPositions.append(ref.offsetted(boundingRect.topRight().rotated(heading, center)));
    //    pathPositions.append(ref.offsetted(boundingRect.bottomRight().rotated(heading, center)));
    //    pathPositions.append(ref.offsetted(boundingRect.bottomLeft().rotated(heading, center)));
    //    pathPositions.append(ref.offsetted(boundingRect.topLeft().rotated(heading, center)));

    m_path = pathPositions;
    emit pathPositionsChanged();
}
