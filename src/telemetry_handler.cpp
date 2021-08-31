#include "telemetry_handler.h"

#include <QDebug>

#include "mavlink_protocol_helpers.h"
#include "mavlink_tmi.h"

using namespace md::domain;

TelemetryHandler::TelemetryHandler(MavlinkHandlerContext* context, QObject* parent) :
    IMavlinkHandler(context, parent),
    m_hasAltitudeMessage(false)
{
}

TelemetryHandler::~TelemetryHandler()
{
}

bool TelemetryHandler::canParse(quint32 msgId)
{
    return (msgId == MAVLINK_MSG_ID_ATTITUDE) || (msgId == MAVLINK_MSG_ID_ALTITUDE) ||
           (msgId == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) || (msgId == MAVLINK_MSG_ID_VFR_HUD) ||
           (msgId == MAVLINK_MSG_ID_GPS_RAW_INT) || (msgId == MAVLINK_MSG_ID_SYS_STATUS) ||
           (msgId == MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT) ||
           (msgId == MAVLINK_MSG_ID_HOME_POSITION);
}

void TelemetryHandler::parseMessage(const mavlink_message_t& message)
{
    // TODO: replace with chain
    if (message.msgid == MAVLINK_MSG_ID_ATTITUDE)
    {
        this->processAttitude(message);
    }
    if (message.msgid == MAVLINK_MSG_ID_ALTITUDE)
    {
        this->processAltitude(message);
    }
    if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
    {
        this->processGlobalPosition(message);
    }
    if (message.msgid == MAVLINK_MSG_ID_VFR_HUD)
    {
        this->processVfrHud(message);
    }
    if (message.msgid == MAVLINK_MSG_ID_GPS_RAW_INT)
    {
        this->processGpsRaw(message);
    }
    if (message.msgid == MAVLINK_MSG_ID_SYS_STATUS)
    {
        this->processSysStatus(message);
    }
    if (message.msgid == MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT)
    {
        this->processNavControllerOutput(message);
    }
    if (message.msgid == MAVLINK_MSG_ID_HOME_POSITION)
    {
        this->processHomePosition(message);
    }
}

void TelemetryHandler::processAttitude(const mavlink_message_t& message)
{
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&message, &attitude);

    m_context->pTree->appendProperties(utils::nodeFromMavId(message.sysid),
                                       {
                                           { tmi::pitch,
                                             utils::fromRadiansToDegrees(attitude.pitch) },
                                           { tmi::roll, utils::fromRadiansToDegrees(attitude.roll) },
                                           { tmi::yaw, utils::fromRadiansToDegrees(attitude.yaw) },
                                       });
}

// FIXME: Ardupilot doesn't send this packet, but we should prioritize this one
void TelemetryHandler::processAltitude(const mavlink_message_t& message)
{
    mavlink_altitude_t altitude;
    mavlink_msg_altitude_decode(&message, &altitude);

    m_hasAltitudeMessage = true;

    m_context->pTree->appendProperties(utils::nodeFromMavId(message.sysid),

                                       { { tmi::altitudeAmsl, altitude.altitude_amsl },
                                         { tmi::altitudeRelative, altitude.altitude_relative },
                                         { tmi::altitudeTerrain, altitude.altitude_terrain } });
}

void TelemetryHandler::processGlobalPosition(const mavlink_message_t& message)
{
    mavlink_global_position_int_t global_position;
    mavlink_msg_global_position_int_decode(&message, &global_position);

    QVariantMap properties({ { tmi::latitude, utils::decodeLatLon(global_position.lat) },
                             { tmi::longitude, utils::decodeLatLon(global_position.lon) },
                             { tmi::heading, utils::fromCentidegrees(global_position.hdg) } });

    if (!m_hasAltitudeMessage)
    {
        properties.insert(tmi::altitudeAmsl, utils::decodeAltitude(global_position.alt));
        properties.insert(tmi::altitudeRelative,
                          utils::decodeAltitude(global_position.relative_alt));
    }

    m_context->pTree->appendProperties(utils::nodeFromMavId(message.sysid), properties);
}

void TelemetryHandler::processSysStatus(const mavlink_message_t& message)
{
    mavlink_sys_status_t sys_status;
    mavlink_msg_sys_status_decode(&message, &sys_status);

    m_context->pTree->appendProperties(utils::nodeFromMavId(message.sysid),

                                       { { tmi::batteryCurrent, sys_status.current_battery },
                                         { tmi::batteryVoltage, sys_status.voltage_battery } });
}

void TelemetryHandler::processHomePosition(const mavlink_message_t& message)
{
    mavlink_home_position_t home_position;
    mavlink_msg_home_position_decode(&message, &home_position);

    m_context->pTree
        ->appendProperties(utils::nodeFromMavId(message.sysid),
                           {
                               { tmi::homeLatitude, utils::decodeLatLon(home_position.latitude) },
                               { tmi::homeLongitude, utils::decodeLatLon(home_position.longitude) },
                               { tmi::homeAltitude, utils::decodeAltitude(home_position.altitude) },
                               // TODO: distance
                           });
}

void TelemetryHandler::processNavControllerOutput(const mavlink_message_t& message)
{
    mavlink_nav_controller_output_t nav_controller_output;
    mavlink_msg_nav_controller_output_decode(&message, &nav_controller_output);

    m_context->pTree->appendProperties(utils::nodeFromMavId(message.sysid),

                                       { { tmi::desiredRoll, nav_controller_output.nav_roll },
                                         { tmi::desiredPitch, nav_controller_output.nav_pitch },
                                         { tmi::desiredHeading, nav_controller_output.nav_bearing },
                                         { tmi::targetBearing,
                                           nav_controller_output.target_bearing },
                                         { tmi::wpDistance, nav_controller_output.wp_dist } });
}

void TelemetryHandler::processVfrHud(const mavlink_message_t& message)
{
    mavlink_vfr_hud_t vfr_hud;
    mavlink_msg_vfr_hud_decode(&message, &vfr_hud);

    m_context->pTree->appendProperties(utils::nodeFromMavId(message.sysid),

                                       { { tmi::ias, vfr_hud.airspeed },
                                         { tmi::tas,
                                           utils::trueAirspeed(vfr_hud.airspeed, vfr_hud.alt) },
                                         { tmi::gs, vfr_hud.groundspeed },
                                         { tmi::climb, vfr_hud.climb },
                                         { tmi::throttle, vfr_hud.throttle } });
}

void TelemetryHandler::processGpsRaw(const mavlink_message_t& message)
{
    mavlink_gps_raw_int_t gps_raw;
    mavlink_msg_gps_raw_int_decode(&message, &gps_raw);

    m_context->pTree->appendProperties(utils::nodeFromMavId(message.sysid),
                                       {
                                           { tmi::satellites, gps_raw.satellites_visible },
                                           // { tmi::gs, decodeGroundSpeed(gps_raw.vel) },
                                           { tmi::course, utils::fromCentidegrees(gps_raw.cog) },
                                       });
}
