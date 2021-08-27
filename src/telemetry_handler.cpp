#include "telemetry_handler.h"

#include "i_property_tree.h"
#include "locator.h"
#include "mavlink_protocol_helpers.h"
#include <math.h>
#include <QDebug>
#include <QJsonArray>
#include <QJsonObject>
#include <QTimerEvent>
#include <QtPositioning/QGeoCoordinate>

using namespace jerom_mavlink::domain;

TelemetryHandler::TelemetryHandler(QObject* parent) :
    IMavlinkHandler(parent),
    m_hasAltitudeMessage(false)
{
    // FIXME: Handlers must not write to pTree directly
    m_pTree = kjarni::domain::Locator::get<kjarni::domain::IPropertyTree>();
    Q_ASSERT(m_pTree);
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
           (msgId == MAVLINK_MSG_ID_MISSION_CURRENT) || (msgId == MAVLINK_MSG_ID_HOME_POSITION);
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
    if (message.msgid == MAVLINK_MSG_ID_MISSION_CURRENT)
    {
        this->processMissionCurrent(message);
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

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject({
                                  { "pitch", ::fromRadiansToDegrees(attitude.pitch) },
                                  { "roll", ::fromRadiansToDegrees(attitude.roll) },
                                  { "yaw", ::fromRadiansToDegrees(attitude.yaw) },
                              }));
}

// TODO: Add new parameters to the https://github.com/Midgrad/kjarni/wiki/Telemetry-parameters

// FIXME: Ardupilot doesn't send this packet, but we should prioritize this one
void TelemetryHandler::processAltitude(const mavlink_message_t& message)
{
    mavlink_altitude_t altitude;
    mavlink_msg_altitude_decode(&message, &altitude);

    m_hasAltitudeMessage = true;

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject({ { "satelliteAltitude", altitude.altitude_amsl },
                                            { "relativeHeight", altitude.altitude_relative },
                                            { "elevation", altitude.altitude_terrain } }));
}

void TelemetryHandler::processGlobalPosition(const mavlink_message_t& message)
{
    mavlink_global_position_int_t global_position;
    mavlink_msg_global_position_int_decode(&message, &global_position);

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject(
                                  { { "latitude", ::decodeLatLon(global_position.lat) },
                                    { "longitude", ::decodeLatLon(global_position.lon) },
                                    { "heading", ::fromCentidegrees(global_position.hdg) } }));

    if (!m_hasAltitudeMessage)
    {
        m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                                  QJsonObject(
                                      { { "satelliteAltitude",
                                          ::decodeAltitude(global_position.alt) },
                                        { "elevation",
                                          ::decodeAltitude(global_position.relative_alt) } }));
    }
}

void TelemetryHandler::processSysStatus(const mavlink_message_t& message)
{
    mavlink_sys_status_t sys_status;
    mavlink_msg_sys_status_decode(&message, &sys_status);

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject({ { "batteryCurrent", sys_status.current_battery },
                                            { "batteryVoltage", sys_status.voltage_battery } }));
}

void TelemetryHandler::processHomePosition(const mavlink_message_t& message)
{
    mavlink_home_position_t home_position;
    mavlink_msg_home_position_decode(&message, &home_position);

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject({
                                  { "homeLatitude", ::decodeLatLon(home_position.latitude) },
                                  { "homeLongitude", ::decodeLatLon(home_position.longitude) },
                                  { "homeAltitude", ::decodeAltitude(home_position.altitude) },

                              }));
}

void TelemetryHandler::processNavControllerOutput(const mavlink_message_t& message)
{
    mavlink_nav_controller_output_t nav_controller_output;
    mavlink_msg_nav_controller_output_decode(&message, &nav_controller_output);

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject(
                                  { { "desiredRoll", nav_controller_output.nav_roll },
                                    { "desiredPitch", nav_controller_output.nav_pitch },
                                    { "desiredHeading", nav_controller_output.nav_bearing },
                                    { "targetBearing", nav_controller_output.target_bearing },
                                    { "wpDistance", nav_controller_output.wp_dist } }));
}

void TelemetryHandler::processMissionCurrent(const mavlink_message_t& message)
{
    mavlink_mission_current_t mission_current;
    mavlink_msg_mission_current_decode(&message, &mission_current);

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject({ { "wp", mission_current.seq } }));
}

void TelemetryHandler::processVfrHud(const mavlink_message_t& message)
{
    mavlink_vfr_hud_t vfr_hud;
    mavlink_msg_vfr_hud_decode(&message, &vfr_hud);

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject({ { "ias", vfr_hud.airspeed },
                                            { "tas", ::trueAirspeed(vfr_hud.airspeed, vfr_hud.alt) },
                                            { "gs", vfr_hud.groundspeed },
                                            { "climb", vfr_hud.climb },
                                            { "throttle", vfr_hud.throttle }

                              }));
}

void TelemetryHandler::processGpsRaw(const mavlink_message_t& message)
{
    mavlink_gps_raw_int_t gps_raw;
    mavlink_msg_gps_raw_int_decode(&message, &gps_raw);

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject({
                                  { "satellites", gps_raw.satellites_visible },
                                  // { "gs", decodeGroundSpeed(gps_raw.vel) },
                                  { "course", ::fromCentidegrees(gps_raw.cog) },

                              }));
}
