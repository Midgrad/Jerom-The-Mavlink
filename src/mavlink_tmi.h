#ifndef MAVLINK_TMI_H
#define MAVLINK_TMI_H

#include "common_tmi.h"

namespace md::tmi
{
// Engine
constexpr char throttle[] = "throttle";

// Battery
constexpr char batteryVoltage[] = "batteryVoltage";
constexpr char batteryCurrent[] = "batteryCurrent";

// SNS
constexpr char satellites[] = "satellites";
} // namespace md::tmi

#endif // MAVLINK_TMI_H
