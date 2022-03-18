#include "mavlink_mission_pattern.h"

#include <QDebug>

#include "mavlink_mission_traits.h"

using namespace md::domain;

namespace
{
const MissionItemType* typeByName(const QString& typeName)
{
    if (typeName == mission::setTriggerDist.name)
        return &mission::setTriggerDist;
    if (typeName == mission::setTriggerInt.name)
        return &mission::setTriggerInt;

    return &mission::waypoint;
}
} // namespace

MavlinkMissionPattern::MavlinkMissionPattern(const RoutePatternType* type,
                                         IRoutePatternAlgorithm* algorithm, QObject* parent) :
    RoutePattern(type, parent),
    m_algorithm(algorithm)
{
}

MavlinkMissionPattern::~MavlinkMissionPattern()
{
}

bool MavlinkMissionPattern::isReady() const
{
    return m_path.positions().length();
}

QList<MissionRouteItem*> MavlinkMissionPattern::createItems()
{
    const MissionItemType* payloadType = ::typeByName(
        this->parameter(mission::surveyType.id).toString());

    QList<MissionRouteItem*> items;
    for (const Geodetic& position : m_path.positions())
    {
        MissionRouteItem* item = new MissionRouteItem(payloadType, payloadType->shortName);
        item->position.set(position);
        items.append(item);
    }
    return items;
}

void MavlinkMissionPattern::calculate()
{
    if (m_area.positions().length() < 3)
    {
        m_path = {};
        emit pathPositionsChanged();
        return;
    }

    const Geodetic ref = m_area.positions().first();
    const auto cartesians = m_algorithm->calculate(m_area.nedPath(ref).positions,
                                                   this->parameters());

    QVector<Geodetic> pathPositions;
    for (const Cartesian& cartesian : cartesians)
    {
        pathPositions.append(ref.offsetted(cartesian));
    }
    m_path = pathPositions;

    emit pathPositionsChanged();
}
