#include "mavlink_mission_factory.h"

using namespace md::domain;

MavlinkMissionFactory::MavlinkMissionFactory()
{
}

Mission* MavlinkMissionFactory::create()
{
    auto mission = new Mission(QVariant(), "New mission");

    return mission;
}
