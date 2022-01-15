#ifndef SURVEY_ROUTE_PATTERN_H
#define SURVEY_ROUTE_PATTERN_H

#include "route_pattern.h"

namespace md::domain
{
class SurveyRoutePattern : public RoutePattern
{
    Q_OBJECT

public:
    explicit SurveyRoutePattern(QObject* parent = nullptr);

    void calculate() override;
};

} // namespace md::domain

#endif // SURVEY_ROUTE_PATTERN_H
