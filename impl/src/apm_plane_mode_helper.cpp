#include "apm_plane_mode_helper.h"

#include <QMap>

using namespace md::data_source;

namespace
{
enum ApmPlaneMode : quint32
{
    MANUAL = 0,
    CIRCLE = 1,
    STABILIZE = 2,
    TRAINING = 3,
    ACRO = 4,
    FBWA = 5,
    FBWB = 6,
    CRUISE = 7,
    AUTOTUNE = 8,
    RESERVED_9 = 9,
    AUTO = 10,
    RTL = 11,
    LOITER = 12,
    RESERVED_13 = 13,
    AVOID = 14,
    GUIDED = 15,
    INITIALIZING = 16,
    QSTABILIZE = 17,
    QHOVER = 18,
    QLOITER = 19,
    QLAND = 20,
    QRTL = 21
};

QMap<quint32, QString> modeMap = { { MANUAL, modes::manual },       { CIRCLE, modes::circle },
                                   { STABILIZE, modes::stabilize }, { ACRO, modes::acro },
                                   { FBWA, modes::fbwa },           { FBWB, modes::fbwb },
                                   { CRUISE, modes::cruise },       { AUTOTUNE, modes::autotune },
                                   { AUTO, modes::mission },        { RTL, modes::rtl },
                                   { LOITER, modes::loiter },       { AVOID, modes::avoid },
                                   { GUIDED, modes::navTo },        { INITIALIZING, modes::init } };

const QStringList availableModes{ //        modes::manual,
                                  //        modes::stabilize,
                                  //        modes::acro,
                                  //        modes::autotune,
                                  //        modes::cruise,
                                  modes::rtl,   modes::mission, modes::circle, modes::loiter,
                                  modes::navTo, modes::fbwa,    modes::fbwb
};
} // namespace

ApmPlaneModeHelper::ApmPlaneModeHelper() : IModeHelper()
{
}

int ApmPlaneModeHelper::modeToCustomMode(const QString& mode)
{
    return ::modeMap.key(mode, -1);
}

QString ApmPlaneModeHelper::customModeToMode(quint32 mode)
{
    return ::modeMap.value(mode);
}

QStringList ApmPlaneModeHelper::availableModes() const
{
    return ::availableModes;
}
