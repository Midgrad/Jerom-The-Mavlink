#include "mavlink_mission_common.h"

#include <QDebug>

#include "mavlink_mission_traits.h"
#include "mavlink_protocol_helpers.h"

using namespace md::domain;

MavlinkMissionCommon::MavlinkMissionCommon(MavlinkHandlerContext* context,
                                           IMissionsService* missionsService, QObject* parent) :
    IMavlinkHandler(context, parent),
    m_missionsService(missionsService)
{
    Q_ASSERT(missionsService);

    connect(missionsService, &IMissionsService::missionAdded, this,
            &MavlinkMissionCommon::onMissionAdded);
    connect(missionsService, &IMissionsService::missionRemoved, this,
            &MavlinkMissionCommon::onMissionRemoved);
}

void MavlinkMissionCommon::parse(const mavlink_message_t& message)
{
    QString vehicleId = m_context->vehicleIds.value(message.sysid).toString();
    if (vehicleId.isNull())
        return;

    switch (message.msgid)
    {
    case MAVLINK_MSG_ID_MISSION_CURRENT:
        return this->processMissionCurrent(message, vehicleId);
    case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
        return this->processMissionReached(message, vehicleId);
    case MAVLINK_MSG_ID_HOME_POSITION:
        return this->processHomePosition(message, vehicleId);
    case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
        return this->processTargetPosition(message, vehicleId);
    default:
        break;
    }
}

void MavlinkMissionCommon::onVehicleObtained(Vehicle* vehicle)
{
    // Check we already have mission
    Mission* mission = m_missionsService->missionForVehicle(vehicle->id);
    if (mission)
        return;

    // Autocrete mission for new vehicle
    mission = new Mission(&mission::mavlinkMissionType, tr("%1 mission").arg(vehicle->name),
                          vehicle->id);
    mission->assignRoute(
        new Route(mission::mavlinkMissionType.routeType, tr("%1 route").arg(vehicle->name)));
    m_missionsService->saveMission(mission);

    // Automaticaly download mission TODO: to settings
    m_missionsService->startOperation(mission, MissionOperation::Download);
}

void MavlinkMissionCommon::processMissionCurrent(const mavlink_message_t& message,
                                                 const QVariant& vehicleId)
{
    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    mavlink_mission_current_t mission_current;
    mavlink_msg_mission_current_decode(&message, &mission_current);

    mission->setCurrentItem(mission_current.seq);
}

void MavlinkMissionCommon::processMissionReached(const mavlink_message_t& message,
                                                 const QVariant& vehicleId)
{
    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    mavlink_mission_item_reached_t reached;
    mavlink_msg_mission_item_reached_decode(&message, &reached);

    mission->setReached(reached.seq);
}

void MavlinkMissionCommon::processHomePosition(const mavlink_message_t& message,
                                               const QVariant& vehicleId)
{
    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    mavlink_home_position_t homePosition;
    mavlink_msg_home_position_decode(&message, &homePosition);

    mission->home()->position.set(Geodetic(utils::decodeLatLon(homePosition.latitude),
                                           utils::decodeLatLon(homePosition.longitude),
                                           utils::decodeAltitude(homePosition.altitude)));
}

void MavlinkMissionCommon::processTargetPosition(const mavlink_message_t& message,
                                                 const QVariant& vehicleId)
{
    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    mavlink_position_target_global_int_t position;
    mavlink_msg_position_target_global_int_decode(&message, &position);

    mission->target()->position.set(Geodetic(utils::decodeLatLon(position.lat_int),
                                             utils::decodeLatLon(position.lon_int), position.alt));
    mission->target()->current.set(true);
}

void MavlinkMissionCommon::sendMissionSetCurrent(const QVariant& vehicleId, int index)
{
    qDebug() << "sendMissionSetCurrent" << vehicleId << index;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
    if (!mavId)
        return;

    mavlink_mission_set_current_t setCurrent;

    setCurrent.target_system = mavId;
    setCurrent.target_component = MAV_COMP_ID_MISSIONPLANNER;
    setCurrent.seq = index;

    mavlink_message_t message;
    mavlink_msg_mission_set_current_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                                &setCurrent); // TODO: link channel
    emit sendMessage(message);
}

void MavlinkMissionCommon::onMissionAdded(Mission* mission)
{
    m_vehicleMissions.insert(mission->vehicleId, mission);

    connect(mission, &Mission::goTo, this, [this, mission](int index) {
        this->sendMissionSetCurrent(mission->vehicleId, index);
    });
}

void MavlinkMissionCommon::onMissionRemoved(Mission* mission)
{
    m_vehicleMissions.remove(mission->vehicleId);

    disconnect(mission, nullptr, this, nullptr);
}
