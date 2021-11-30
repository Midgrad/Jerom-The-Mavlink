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

void SystemStatusHandler::parse(const mavlink_message_t& message)
{
    if (message.msgid == MAVLINK_MSG_ID_SYS_STATUS)
        this->processSystemStatus(message);
}

void SystemStatusHandler::processSystemStatus(const mavlink_message_t& message)
{
    QString vehicleId = m_context->vehicleIds.value(message.sysid).toString();
    if (vehicleId.isEmpty())
        return;

    mavlink_sys_status_t sysStatus;
    mavlink_msg_sys_status_decode(&message, &sysStatus);

    QVariantList devices;

    QVariantMap ahrs;
    ahrs[tmi::present] = sysStatus.onboard_control_sensors_present & MAV_SYS_STATUS_AHRS;
    ahrs[tmi::enabled] = sysStatus.onboard_control_sensors_enabled & MAV_SYS_STATUS_AHRS;
    ahrs[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_AHRS;
    ahrs[tmi::name] = tmi::ahrs;
    devices.append(ahrs);

    QVariantMap accel;
    accel[tmi::present] = sysStatus.onboard_control_sensors_present &
                          MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    accel[tmi::enabled] = sysStatus.onboard_control_sensors_enabled &
                          MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    accel[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    accel[tmi::name] = tmi::accel;
    devices.append(accel);

    QVariantMap gyro;
    gyro[tmi::present] = sysStatus.onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_3D_GYRO;
    gyro[tmi::enabled] = sysStatus.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_GYRO;
    gyro[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_GYRO;
    gyro[tmi::name] = tmi::gyro;
    devices.append(gyro);

    QVariantMap mag;
    mag[tmi::present] = sysStatus.onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_3D_MAG;
    mag[tmi::enabled] = sysStatus.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_MAG;
    mag[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG;
    mag[tmi::name] = tmi::mag;
    devices.append(mag);

    QVariantMap gps;
    gps[tmi::present] = sysStatus.onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_GPS;
    gps[tmi::enabled] = sysStatus.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_GPS;
    gps[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_GPS;
    gps[tmi::name] = tmi::gps;
    devices.append(gps);

    QVariantMap baro;
    baro[tmi::present] = sysStatus.onboard_control_sensors_present &
                         MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    baro[tmi::enabled] = sysStatus.onboard_control_sensors_enabled &
                         MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    baro[tmi::health] = sysStatus.onboard_control_sensors_health &
                        MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    baro[tmi::name] = tmi::baro;
    devices.append(baro);

    QVariantMap pitot;
    pitot[tmi::present] = sysStatus.onboard_control_sensors_present &
                          MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    pitot[tmi::enabled] = sysStatus.onboard_control_sensors_enabled &
                          MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    pitot[tmi::health] = sysStatus.onboard_control_sensors_health &
                         MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
    pitot[tmi::name] = tmi::pitot;
    devices.append(pitot);

    QVariantMap radalt;
    radalt[tmi::present] = sysStatus.onboard_control_sensors_present &
                           MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    radalt[tmi::enabled] = sysStatus.onboard_control_sensors_enabled &
                           MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    radalt[tmi::health] = sysStatus.onboard_control_sensors_health &
                          MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    radalt[tmi::name] = tmi::radalt;
    devices.append(radalt);

    QVariantMap battery;
    battery[tmi::present] = sysStatus.onboard_control_sensors_present &
                            MAV_SYS_STATUS_SENSOR_BATTERY;
    battery[tmi::enabled] = sysStatus.onboard_control_sensors_enabled &
                            MAV_SYS_STATUS_SENSOR_BATTERY;
    battery[tmi::health] = sysStatus.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_BATTERY;
    battery[tmi::name] = tmi::battery;
    devices.append(battery);

    QVariantMap status;
    status[tmi::devices] = devices;
    status[tmi::batteryCurrent] = utils::decodeCurrent(sysStatus.current_battery);
    status[tmi::batteryVoltage] = utils::decodeVoltage(sysStatus.voltage_battery);
    status[tmi::batteryPercentage] = sysStatus.battery_remaining;

    // TODO: load, drop rate, errors

    m_context->pTree->appendProperties(vehicleId, status); // TODO: remove props pTree from context
}
