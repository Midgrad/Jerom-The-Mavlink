#include "module_jerom_the_mavlink.h"

#include <QDebug>

#include "locator.h"

#include "communication_service.h"
#include "i_vehicles_features.h"
#include "mavlink_handlers_factory.h"
#include "mavlink_mission_traits.h"
#include "mavlink_protocol.h"
#include "mavlink_protocol_threaded.h"
#include "mavlink_vehicle_traits.h"

namespace
{
constexpr char protocolName[] = "Mavlink";

constexpr char mavlinkDashboard[] = "MavlinkDashboard.qml";
} // namespace

using namespace md::app;

ModuleJeromTheMavlink::ModuleJeromTheMavlink()
{
}

ModuleJeromTheMavlink::~ModuleJeromTheMavlink()
{
    delete m_protocol;
}

void ModuleJeromTheMavlink::init()
{
    auto pTree = Locator::get<domain::IPropertyTree>();
    Q_ASSERT(pTree);

    auto vehiclesService = Locator::get<domain::IVehiclesService>();
    Q_ASSERT(vehiclesService);

    auto vehiclesFeatures = Locator::get<domain::IVehiclesFeatures>();
    Q_ASSERT(vehiclesFeatures);

    for (const domain::VehicleType* type : domain::vehicle::allMavlinkTypes)
    {
        vehiclesService->addVehicleType(type);
        vehiclesFeatures->addFeature(type->id, domain::features::dashboard, ::mavlinkDashboard);
    }

    auto missionService = Locator::get<domain::IMissionsService>();
    Q_ASSERT(missionService);

    auto commandsService = Locator::get<domain::ICommandsService>();
    Q_ASSERT(commandsService);

    auto communicationService = Locator::get<CommunicationService>();
    Q_ASSERT(communicationService);

    missionService->registerMissionType(&domain::mission::mavlinkMissionType);

    domain::MavlinkHandlerFactory factory(pTree, missionService, vehiclesService, commandsService);

    m_protocol = new data_source::MavlinkProtocolThreaded(new data_source::MavlinkProtocol(&factory,
                                                                                           nullptr),
                                                          this);

    communicationService->registerProtocol(protocolName, m_protocol);
}

void ModuleJeromTheMavlink::start()
{
}

void ModuleJeromTheMavlink::done()
{
    auto vehiclesFeatures = Locator::get<domain::IVehiclesFeatures>();
    Q_ASSERT(vehiclesFeatures);
    for (const domain::VehicleType* type : domain::vehicle::allMavlinkTypes)
    {
        vehiclesFeatures->removeFeatures(type->id);
    }

    auto missionService = Locator::get<domain::IMissionsService>();
    Q_ASSERT(missionService);
    missionService->unregisterMissionType(&domain::mission::mavlinkMissionType);
}
