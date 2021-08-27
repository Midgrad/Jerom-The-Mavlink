#ifndef APM_PLANE_MODE_HELPER_H
#define APM_PLANE_MODE_HELPER_H

#include "i_mode_helper.h"

namespace jerom_mavlink::data_source
{
class ApmPlaneModeHelper : public IModeHelper
{
public:
    ApmPlaneModeHelper();

    int modeToCustomMode(const QString& mode) override;
    QString customModeToMode(quint32 mode) override;

    virtual QStringList availableModes() const override;
};
} // namespace jerom_mavlink::data_source

#endif // APM_PLANE_MODE_HELPER_H
