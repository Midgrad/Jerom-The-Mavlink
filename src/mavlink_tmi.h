#ifndef MAVLINK_TMI_H
#define MAVLINK_TMI_H

#include "common_tmi.h"

namespace md::tmi
{
// Devices
constexpr char ahrs[] = "ahrs";
constexpr char accel[] = "accel";
constexpr char gyro[] = "gyro";
constexpr char mag[] = "mag";
constexpr char gps[] = "gps";
constexpr char baro[] = "baro";
constexpr char pitot[] = "pitot";
constexpr char radalt[] = "radalt";
constexpr char battery[] = "battery";

// Engine
constexpr char throttle[] = "throttle";

// Battery
constexpr char batteryPercentage[] = "batteryPercentage";
constexpr char batteryVoltage[] = "batteryVoltage";
constexpr char batteryCurrent[] = "batteryCurrent";

// SNS
constexpr char satellites[] = "satellites";
} // namespace md::tmi

#endif // MAVLINK_TMI_H
