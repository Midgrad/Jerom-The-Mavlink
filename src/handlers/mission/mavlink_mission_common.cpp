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
    mission = new Mission(&mission::mavlinkMission, tr("%1 mission").arg(vehicle->name),
                          vehicle->id);
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

    mission->route()->setCurrent(mission_current.seq);
}

void MavlinkMissionCommon::processMissionReached(const mavlink_message_t& message,
                                                 const QVariant& vehicleId)
{
    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    mavlink_mission_item_reached_t reached;
    mavlink_msg_mission_item_reached_decode(&message, &reached);

    mission->route()->setReached(reached.seq);
}

void MavlinkMissionCommon::processHomePosition(const mavlink_message_t& message,
                                               const QVariant& vehicleId)
{
    Mission* mission = m_vehicleMissions.value(vehicleId, nullptr);
    if (!mission)
        return;

    MissionRouteItem* home = mission->route()->firstItem();
    if (!home)
        return;

    mavlink_home_position_t homePosition;
    mavlink_msg_home_position_decode(&message, &homePosition);

    home->position.set(Geodetic(utils::decodeLatLon(homePosition.latitude),
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

    // TODO: trerget vehicle
    //    mission->target()->position.set(Geodetic(utils::decodeLatLon(position.lat_int),
    //                                             utils::decodeLatLon(position.lon_int), position.alt));
    //    mission->target()->current.set(true);
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

void MavlinkMissionCommon::sendNavTo(const QVariant& vehicleId, double latitude, double longitude,
                                     float altitude)
{
    qDebug() << "sendNavTo" << vehicleId << latitude << longitude << altitude;
    auto mavId = m_context->vehicleIds.key(vehicleId, 0);
    if (!mavId)
        return;

    mavlink_message_t message;
    mavlink_mission_item_t item;

    item.param1 = 0;
    item.param2 = 0;
    item.param3 = 0;
    item.param4 = 0;
    item.target_system = mavId;
    item.target_component = MAV_COMP_ID_MISSIONPLANNER;
    item.frame = MAV_FRAME_GLOBAL_INT;
    item.command = MAV_CMD_NAV_WAYPOINT;
    item.current = 2; // guided
    item.seq = 0;
    item.autocontinue = false;
    item.x = static_cast<float>(latitude);
    item.y = static_cast<float>(longitude);
    item.z = altitude;

    mavlink_msg_mission_item_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                         &item); // TODO: link channel

    emit sendMessage(message);
}

void MavlinkMissionCommon::onMissionAdded(Mission* mission)
{
    m_vehicleMissions.insert(mission->vehicleId, mission);

    connect(mission->route, &MissionRoute::goTo, this, [this, mission](int index) {
        this->sendMissionSetCurrent(mission->vehicleId, index);
    });
    connect(mission, &Mission::navTo, this,
            [this, mission](double latitude, double longitude, float altitude) {
                this->sendNavTo(mission->vehicleId, latitude, longitude, altitude);
            });
}

void MavlinkMissionCommon::onMissionRemoved(Mission* mission)
{
    m_vehicleMissions.remove(mission->vehicleId);

    disconnect(mission, nullptr, this, nullptr);
}
