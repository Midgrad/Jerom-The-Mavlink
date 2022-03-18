#ifndef MAVLINK_MISSION_PATTERN_H
#define MAVLINK_MISSION_PATTERN_H

#include "mission_pattern.h"

#include "i_mission_pattern_algorithm.h"

namespace md::domain
{
class MavlinkMissionPattern : public RoutePattern
{
    Q_OBJECT

public:
    explicit MavlinkMissionPattern(const RoutePatternType* type, IRoutePatternAlgorithm* algorithm,
                                   QObject* parent = nullptr);
    ~MavlinkMissionPattern() override;

    bool isReady() const override;
    QList<MissionRouteItem*> createItems() override;

    void calculate() override;

private:
    const QScopedPointer<IRoutePatternAlgorithm> m_algorithm;
};

} // namespace md::domain

#endif // MAVLINK_MISSION_PATTERN_H
