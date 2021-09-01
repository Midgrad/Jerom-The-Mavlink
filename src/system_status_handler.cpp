#include "system_status_handler.h"

#include <QDebug>

#include "mavlink_protocol_helpers.h"
#include "mavlink_tmi.h"

using namespace md::domain;

SystemStatusHandler::SystemStatusHandler(MavlinkHandlerContext* context, QObject* parent) :
    IMavlinkHandler(context, parent)
{
}

SystemStatusHandler::~SystemStatusHandler()
{
}

bool SystemStatusHandler::canParse(quint32 msgId)
{
    return msgId == MAVLINK_MSG_ID_SYS_STATUS;
}

void SystemStatusHandler::parseMessage(const mavlink_message_t& message)
{
    if (message.msgid == MAVLINK_MSG_ID_SYS_STATUS)
    {
        this->processSystemStatus(message);
    }
}

void SystemStatusHandler::processSystemStatus(const mavlink_message_t& message)
{
    mavlink_sys_status_t sysStatus;
    mavlink_msg_sys_status_decode(&message, &sysStatus);

    QVariantMap status;

    QVariantMap ahrs;
    ahrs[tmi::present] = sysStatus.onboard_control_sensors_present & MAV_SYS_STATUS_AHRS;
    ahrs[tmi::enabled] = sysStatus.onboard_control_sensors_enabled & MAV_SYS_STATUS_AHRS;
    ahrs[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_AHRS;
    status[tmi::ahrs] = ahrs;

    QVariantMap accel;
    accel[tmi::present] = sysStatus.onboard_control_sensors_present &
                          MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    accel[tmi::enabled] = sysStatus.onboard_control_sensors_enabled &
                          MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    accel[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    status[tmi::accel] = accel;

    QVariantMap gyro;
    gyro[tmi::present] = sysStatus.onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_3D_GYRO;
    gyro[tmi::enabled] = sysStatus.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_GYRO;
    gyro[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_GYRO;
    status[tmi::gyro] = gyro;

    QVariantMap mag;
    mag[tmi::present] = sysStatus.onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_3D_MAG;
    mag[tmi::enabled] = sysStatus.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_MAG;
    mag[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG;
    status[tmi::mag] = mag;

    QVariantMap gps;
    gps[tmi::present] = sysStatus.onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_GPS;
    gps[tmi::enabled] = sysStatus.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_GPS;
    gps[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_GPS;
    status[tmi::gps] = gps;

    QVariantMap baro;
    baro[tmi::present] = sysStatus.onboard_control_sensors_present &
                         MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    baro[tmi::enabled] = sysStatus.onboard_control_sensors_enabled &
                         MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    baro[tmi::health] = sysStatus.onboard_control_sensors_health &
                        MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    status[tmi::baro] = baro;

    QVariantMap pitot;
    pitot[tmi::present] = sysStatus.onboard_control_sensors_present &
                          MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    pitot[tmi::enabled] = sysStatus.onboard_control_sensors_enabled &
                          MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    pitot[tmi::health] = sysStatus.onboard_control_sensors_health &
                         MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    status[tmi::pitot] = pitot;

    QVariantMap radalt;
    radalt[tmi::present] = sysStatus.onboard_control_sensors_present &
                           MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    radalt[tmi::enabled] = sysStatus.onboard_control_sensors_enabled &
                           MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    radalt[tmi::health] = sysStatus.onboard_control_sensors_health &
                          MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    status[tmi::radalt] = radalt;

    QVariantMap battery;
    battery[tmi::present] = sysStatus.onboard_control_sensors_present &
                            MAV_SYS_STATUS_SENSOR_BATTERY;
    battery[tmi::enabled] = sysStatus.onboard_control_sensors_enabled &
                            MAV_SYS_STATUS_SENSOR_BATTERY;
    battery[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_BATTERY;
    status[tmi::battery] = battery;

    status[tmi::batteryCurrent] = utils::decodeCurrent(sysStatus.current_battery);
    status[tmi::batteryVoltage] = utils::decodeVoltage(sysStatus.voltage_battery);
    status[tmi::batteryPercentage] = sysStatus.battery_remaining;

    // TODO: load, drop rate, errors

    m_context->pTree->appendProperties(utils::nodeFromMavId(message.sysid), status);
}
