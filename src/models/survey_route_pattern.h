#ifndef SURVEY_ROUTE_PATTERN_H
#define SURVEY_ROUTE_PATTERN_H

#include "route_pattern.h"

#include "i_route_pattern_algorithm.h"

namespace md::domain
{
class SurveyRoutePattern : public RoutePattern
{
    Q_OBJECT

public:
    explicit SurveyRoutePattern(IRoutePatternAlgorithm* algorithm, QObject* parent = nullptr);
    ~SurveyRoutePattern() override;

    void calculate() override;

private:
    const QScopedPointer<IRoutePatternAlgorithm> m_algorithm;
};

} // namespace md::domain

#endif // SURVEY_ROUTE_PATTERN_H
