#ifndef I_MODE_HELPER_H
#define I_MODE_HELPER_H

#include "mode_traits.h"

namespace jerom_mavlink::data_source
{
class IModeHelper
{
public:
    IModeHelper() = default;
    virtual ~IModeHelper() = default;

    virtual int modeToCustomMode(const QString& mode) = 0;
    virtual QString customModeToMode(quint32 mode) = 0;

    virtual QStringList availableModes() const = 0;
};
} // namespace jerom_mavlink::data_source

#endif // I_MODE_HELPER_H
