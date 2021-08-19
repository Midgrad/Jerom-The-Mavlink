#include "telemetry_handler.h"

#include "QJsonObject"
#include "i_property_tree.h"
#include "locator.h"
#include <QDebug>
#include <QJsonArray>
#include <QTimerEvent>

using namespace jerom_mavlink;

TelemetryHandler::TelemetryHandler()
{
}

TelemetryHandler::~TelemetryHandler()
{
}

bool TelemetryHandler::canParse(quint32 msgId)
{
    return (msgId == MAVLINK_MSG_ID_ATTITUDE) || (msgId == MAVLINK_MSG_ID_ALTITUDE) ||
           (msgId == MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
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
}

void TelemetryHandler::processAttitude(const mavlink_message_t& message)
{
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&message, &attitude);

    qDebug() << "pitch: " << attitude.pitch;
    qDebug() << "roll: " << attitude.roll;
    qDebug() << "yaw: " << attitude.yaw;
}

void TelemetryHandler::processAltitude(const mavlink_message_t& message)
{
    mavlink_altitude_t altitude;
    mavlink_msg_altitude_decode(&message, &altitude);

    qDebug() << "altitude: " << altitude.altitude_amsl;
}

void TelemetryHandler::processGlobalPosition(const mavlink_message_t& message)
{
    mavlink_global_position_int_t global_position;
    mavlink_msg_global_position_int_decode(&message, &global_position);

    kjarni::domain::IPropertyTree* pTree =
        kjarni::domain::Locator::get<kjarni::domain::IPropertyTree>();
    Q_ASSERT(pTree);
    QJsonArray modes = QJsonArray({ "auto", "manual", "rtl", "circle" });
    pTree->setProperty("MAV 23",
                       QJsonObject(
                           { { "callsign", "MAVIK 23" },
                             { "state", "ACTIVE" },
                             { "armed", true },
                             { "mode", "auto" },
                             { "modes", modes },
                             { "gs", 34.234 },
                             { "ias", 36.875 },
                             { "tas", 36.963 },
                             { "pitch", 12.2 },
                             { "roll", 23.6 },
                             { "latitude", static_cast<float>(global_position.lat) / 10000000 },
                             { "longitude", static_cast<float>(global_position.lon) / 10000000 },
                             { "satelliteAltitude", 5657 },
                             { "relativeHeight", 5544 },
                             { "absoluteHeight", 5660 },
                             { "climb", -1.1 },
                             { "elevation", 5324 },
                             { "heading", 132.7 },
                             { "course", 141.2 },
                             { "wp", 3 },
                             { "wps", 28 },
                             { "wpDistance", 1453 },
                             { "homeDistance", 2315 } }));

    qDebug() << "lat: " << static_cast<float>(global_position.lat) / 10000000;
    qDebug() << "lon: " << static_cast<float>(global_position.lon) / 10000000;
    qDebug() << "alt: " << global_position.alt;
}
