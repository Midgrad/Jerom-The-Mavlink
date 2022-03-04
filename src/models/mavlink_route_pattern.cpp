//#include "mavlink_route_pattern.h"

//#include <QDebug>

//#include "mavlink_mission_traits.h"

//using namespace md::domain;

//namespace
//{
//const RouteItemType* typeByName(const QString& typeName)
//{
//    if (typeName == route::setTriggerDist.name)
//        return &route::setTriggerDist;
//    if (typeName == route::setTriggerInt.name)
//        return &route::setTriggerInt;

//    return &route::waypoint;
//}
//} // namespace

//MavlinkRoutePattern::MavlinkRoutePattern(const RoutePatternType* type,
//                                         IRoutePatternAlgorithm* algorithm, QObject* parent) :
//    RoutePattern(type, parent),
//    m_algorithm(algorithm)
//{
//}

//MavlinkRoutePattern::~MavlinkRoutePattern()
//{
//}

//bool MavlinkRoutePattern::isReady() const
//{
//    return m_path.positions().length();
//}

//QList<RouteItem*> MavlinkRoutePattern::createItems()
//{
//    const RouteItemType* payloadType = ::typeByName(
//        this->parameter(route::surveyType.id).toString());

//    QList<RouteItem*> items;
//    for (const Geodetic& position : m_path.positions())
//    {
//        RouteItem* item = new RouteItem(payloadType, payloadType->shortName);
//        item->position.set(position);
//        items.append(item);
//    }
//    return items;
//}

//void MavlinkRoutePattern::calculate()
//{
//    if (m_area.positions().length() < 3)
//    {
//        m_path = {};
//        emit pathPositionsChanged();
//        return;
//    }

//    const Geodetic ref = m_area.positions().first();
//    const auto cartesians = m_algorithm->calculate(m_area.nedPath(ref).positions,
//                                                   this->parameters());

//    QVector<Geodetic> pathPositions;
//    for (const Cartesian& cartesian : cartesians)
//    {
//        pathPositions.append(ref.offsetted(cartesian));
//    }
//    m_path = pathPositions;

//    emit pathPositionsChanged();
//}
