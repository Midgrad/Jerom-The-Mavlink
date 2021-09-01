#ifndef PX4_MODE_HELPER_H
#define PX4_MODE_HELPER_H

#include "i_mode_helper.h"

namespace md::data_source
{
class Px4ModeHelper : public IModeHelper // TODO: separate copter and plane modes
{
public:
    Px4ModeHelper();

    int modeToCustomMode(const QString& mode) override;
    QString customModeToMode(quint32 mode) override;

    QStringList availableModes() const override;
};
} // namespace md::data_source

#endif // PX4_MODE_HELPER_H
