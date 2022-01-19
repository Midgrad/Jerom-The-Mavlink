#ifndef MAVLINK_ROUTE_PATTERN_H
#define MAVLINK_ROUTE_PATTERN_H

#include "route_pattern.h"

#include "i_route_pattern_algorithm.h"

namespace md::domain
{
class MavlinkRoutePattern : public RoutePattern
{
    Q_OBJECT

public:
    explicit MavlinkRoutePattern(const RoutePatternType* type, IRoutePatternAlgorithm* algorithm,
                                 QObject* parent = nullptr);
    ~MavlinkRoutePattern() override;

    bool isReady() const override;
    QList<RouteItem*> items() override;

    void calculate() override;

private:
    const QScopedPointer<IRoutePatternAlgorithm> m_algorithm;
};

} // namespace md::domain

#endif // MAVLINK_ROUTE_PATTERN_H
