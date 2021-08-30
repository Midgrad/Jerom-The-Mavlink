#ifndef APM_COPTER_MODE_HELPER_H
#define APM_COPTER_MODE_HELPER_H

#include "i_mode_helper.h"

namespace md::data_source
{
class ApmCopterModeHelper : public IModeHelper
{
public:
    ApmCopterModeHelper();

    int modeToCustomMode(const QString& mode) override;
    QString customModeToMode(quint32 mode) override;

    virtual QStringList availableModes() const override;
};
} // namespace jerom_mavlink::data_source

#endif // APM_COPTER_MODE_HELPER_H
