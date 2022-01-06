#ifndef MAVLINK_VEHICLE_TRAITS_H
#define MAVLINK_VEHICLE_TRAITS_H

#include "vehicle_traits.h"

namespace md::domain
{
namespace vehicle
{
constexpr char mavlinkFixedWing[] = "Mavlink Fixed Wing";
constexpr char mavlinkCopter[] = "Mavlink Copter";
constexpr char mavlinkRotaryWing[] = "Mavlink Rotary Wing";
constexpr char mavlinkVtol[] = "Mavlink VTOL";
constexpr char mavlinkAirship[] = "Mavlink Airship";
} // namespace vehicle
} // namespace md::domain

#endif // MAVLINK_VEHICLE_TRAITS_H
