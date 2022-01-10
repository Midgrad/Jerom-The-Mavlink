#ifndef MAVLINK_VEHICLE_TRAITS_H
#define MAVLINK_VEHICLE_TRAITS_H

#include "vehicle_traits.h"

namespace md::domain
{
namespace vehicle
{
const VehicleType mavlinkFixedWing = { "mavlink_fixed_wing", "Mavlink Fixed Wing", "fixed_wing.svg",
                                       "fixed_wing.glb" };
const VehicleType mavlinkCopter = { "mavlink_copter", "Mavlink Copter", "copter.svg", "copter.glb" };
const VehicleType mavlinkRotaryWing = { "mavlink_rotary_wing", "Mavlink Rotary Wing",
                                        "rotary_wing.svg", "rotary_wing.glb" };
const VehicleType mavlinkVtol = { "mavlink_vtol", "Mavlink VTOL", "vtol.svg", "vtol.glb" };
const VehicleType mavlinkAirship = { "mavlink_airship", "Mavlink Airship", "airship.svg",
                                     "airship.glb" };

const QList<const VehicleType*> allMavlinkTypes = { &mavlinkFixedWing, &mavlinkCopter,
                                                    &mavlinkRotaryWing, &mavlinkVtol,
                                                    &mavlinkAirship };
} // namespace vehicle
} // namespace md::domain

#endif // MAVLINK_VEHICLE_TRAITS_H
