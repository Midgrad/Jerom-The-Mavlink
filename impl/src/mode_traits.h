#ifndef MODE_TRAITS_H
#define MODE_TRAITS_H

#include <QString>

namespace md::data_source
{
namespace modes
{
constexpr char none[] = "None";
constexpr char init[] = "Init";
constexpr char manual[] = "Manual";
constexpr char stabilize[] = "Stabilize";
constexpr char acro[] = "Acro";
constexpr char fbwa[] = "Fbwa";
constexpr char fbwb[] = "Fbwb";
constexpr char rattitude[] = "Rattitude";
constexpr char posCtrl[] = "PosCtrl";
constexpr char altCtrl[] = "AltCtrl";
constexpr char altHold[] = "AltHold";
constexpr char holdPosition[] = "HoldPosition";
constexpr char cruise[] = "Cruise";
constexpr char autotune[] = "Autotune";
constexpr char navTo[] = "NavTo";
constexpr char loiter[] = "Loiter";
constexpr char circle[] = "Circle";
constexpr char rtl[] = "RTL";
constexpr char takeoff[] = "Takeoff";
constexpr char landing[] = "Landing";
constexpr char mission[] = "Mission";
constexpr char throwing[] = "Throw";
constexpr char avoid[] = "Avoid";
constexpr char follow[] = "Follow";
} // namespace modes
} // namespace md::data_source

#endif // MODE_TRAITS_H
