#include "telemetry_handler.h"

#include "i_property_tree.h"
#include "locator.h"
#include "mavlink_protocol_helpers.h"
#include <math.h>
#include <QDebug>
#include <QJsonArray>
#include <QJsonObject>
#include <QTimerEvent>

using namespace jerom_mavlink;

TelemetryHandler::TelemetryHandler()
{
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
           (msgId == MAVLINK_MSG_ID_GPS_RAW_INT) || (msgId == MAVLINK_MSG_ID_SYS_STATUS);
}

void TelemetryHandler::parseMessage(const mavlink_message_t& message)
{
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
}

void TelemetryHandler::processAttitude(const mavlink_message_t& message)
{
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&message, &attitude);

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject({
                                  { "pitch", fromRadiansToDegrees(attitude.pitch) },
                                  { "roll", fromRadiansToDegrees(attitude.roll) },
                              }));
}

// FIXME: Arduplane doesn't send this packet
void TelemetryHandler::processAltitude(const mavlink_message_t& message)
{
    mavlink_altitude_t altitude;
    mavlink_msg_altitude_decode(&message, &altitude);

    m_pTree->appendProperties(
        QStringLiteral("MAV %1").arg(message.sysid),
        QJsonObject(
            { //                                      { "latitude", decodeLatLon(altitude.altitude_amsl) },
              { "relativeHeight", decodeLatLon(altitude.altitude_relative) },
              { "elevation", decodeAltitude(altitude.altitude_terrain) } }));
}

void TelemetryHandler::processGlobalPosition(const mavlink_message_t& message)
{
    mavlink_global_position_int_t global_position;
    mavlink_msg_global_position_int_decode(&message, &global_position);

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject(
                                  { { "latitude", decodeLatLon(global_position.lat) },
                                    { "longitude", decodeLatLon(global_position.lon) },
                                    { "satelliteAltitude", decodeAltitude(global_position.alt) },
                                    { "heading", fromCentidegrees(global_position.hdg) } }));
}

// TODO: Add parameters to the https://github.com/Midgrad/kjarni/wiki/Telemetry-parameters
void TelemetryHandler::processSysStatus(const mavlink_message_t& message)
{
    mavlink_sys_status_t sys_status;
    mavlink_msg_sys_status_decode(&message, &sys_status);

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject({
                                  { "batteryCurrent", sys_status.current_battery },
                                  { "batteryVoltage", sys_status.voltage_battery },

                              }));
}

//void TelemetryHandler::processHomePosition(const mavlink_message_t& message)
//{
//    mavlink_home_position_t home_position;
//    mavlink_msg_home_position_decode(&message, &home_position);
//
//    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid), QJsonObject({
//                                            { "home", home_position.},
//                                        }));
//}

void TelemetryHandler::processVfrHud(const mavlink_message_t& message)
{
    mavlink_vfr_hud_t vfr_hud;
    mavlink_msg_vfr_hud_decode(&message, &vfr_hud);

    m_pTree->appendProperties(QStringLiteral("MAV %1").arg(message.sysid),
                              QJsonObject(
                                  { { "ias", vfr_hud.airspeed },
                                    { "tas", getTrueAirspeed(vfr_hud.airspeed, vfr_hud.alt) },
                                    // { "gs", vfr_hud.groundspeed },
                                    // { "alt", vfr_hud.alt },
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
                                  { "gs", decodeGroundSpeed(gps_raw.vel) },
                                  { "course", fromCentidegrees(gps_raw.cog) },

                              }));
}
