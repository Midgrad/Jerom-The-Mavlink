#ifndef MAVLINK_VEHICLE_TRAITS_H
#define MAVLINK_VEHICLE_TRAITS_H

#include "vehicle_traits.h"

namespace md::domain
{
namespace vehicleType
{
constexpr char mavlinkFixedWing[] = "Mavlink Fixed Wing";
constexpr char mavlinkCopter[] = "Mavlink Copter";
constexpr char mavlinkRotaryWing[] = "Mavlink Rotary Wing";
constexpr char mavlinkVtol[] = "Mavlink VTOL";
constexpr char mavlinkAirship[] = "Mavlink Airship";

const std::array<const char*, 5> allMavlinkTypes = { mavlinkFixedWing, mavlinkCopter,
                                                     mavlinkRotaryWing, mavlinkVtol,
                                                     mavlinkAirship };
} // namespace vehicleType
} // namespace md::domain

#endif // MAVLINK_VEHICLE_TRAITS_H
