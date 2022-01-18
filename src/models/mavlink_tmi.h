#ifndef MAVLINK_TMI_H
#define MAVLINK_TMI_H

#include "vehicle_tmi.h"

namespace md::tmi
{
// Devices
constexpr char ahrs[] = "AHRS";
constexpr char accel[] = "Accel";
constexpr char gyro[] = "Gyro";
constexpr char mag[] = "Mag";
constexpr char gps[] = "GPS";
constexpr char baro[] = "Baro";
constexpr char pitot[] = "Pitot";
constexpr char radalt[] = "Radalt";
constexpr char battery[] = "Battery";

// Engine
constexpr char throttle[] = "throttle";

// Battery
constexpr char batteryPercentage[] = "batteryPercentage";
constexpr char batteryVoltage[] = "batteryVoltage";
constexpr char batteryCurrent[] = "batteryCurrent";

// SNS
constexpr char gpsFix[] = "gpsFix";
constexpr char satellitesVisible[] = "satellitesVisible";
} // namespace md::tmi

#endif // MAVLINK_TMI_H
