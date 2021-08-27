#include "apm_copter_mode_helper.h"

#include <common/mavlink.h>

#include <QMap>

using namespace jerom_mavlink::data_source;

namespace
{
enum ApmCopterMode : quint32
{
    STABILIZE = 0,
    ACRO = 1,
    ALT_HOLD = 2,
    AUTO = 3,
    GUIDED = 4,
    LOITER = 5,
    RTL = 6,
    CIRCLE = 7,
    POSITION = 8,
    LAND = 9,
    OF_LOITER = 10,
    DRIFT = 11,
    RESERVED_12 = 12,
    SPORT = 13,
    FLIP = 14,
    AUTOTUNE = 15,
    POS_HOLD = 16,
    BRAKE = 17,
    THROW = 18,
    AVOID_ADSB = 19,
    GUIDED_NOGPS = 20,
    SAFE_RTL = 21
};

QMap<quint32, QString> modeMap = { { STABILIZE, modes::stabilize },
                                   { ACRO, modes::acro },
                                   { ALT_HOLD, modes::altHold },
                                   { AUTO, modes::mission },
                                   { GUIDED, modes::navTo },
                                   { LOITER, modes::loiter },
                                   { RTL, modes::rtl },
                                   { CIRCLE, modes::circle },
                                   { LAND, modes::landing },
                                   { AUTOTUNE, modes::autotune },
                                   { POS_HOLD, modes::holdPosition },
                                   { THROW, modes::throwing },
                                   { AVOID_ADSB, modes::avoid } };

const QStringList availableModes{ modes::rtl, modes::mission, modes::loiter, modes::altHold,
                                  modes::stabilize };
} // namespace

ApmCopterModeHelper::ApmCopterModeHelper() : IModeHelper()
{
}

int ApmCopterModeHelper::modeToCustomMode(const QString& mode)
{
    return ::modeMap.key(mode, -1);
}

QString ApmCopterModeHelper::customModeToMode(quint32 mode)
{
    return ::modeMap.value(mode);
}

QStringList ApmCopterModeHelper::availableModes() const
{
    return ::availableModes;
}
