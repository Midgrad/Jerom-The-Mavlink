#include "telemetry_handler.h"

#include "i_property_tree.h"
#include "locator.h"
#include <math.h>
#include <QDebug>
#include <QJsonArray>
#include <QJsonObject>
#include <QTimerEvent>

using namespace jerom_mavlink::domain;

TelemetryHandler::TelemetryHandler(QObject* parent) : IMavlinkHandler(parent)
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

    m_pTree->appendProperties("MAV 23", QJsonObject({ { "pitch", attitude.pitch * 180 / M_PI },
                                                      { "roll", attitude.roll * 180 / M_PI },
                                                      { "heading", attitude.yaw * 180 / M_PI } }));
}

// FIXME: Arduplane doesn't send this packet
void TelemetryHandler::processAltitude(const mavlink_message_t& message)
{
    mavlink_altitude_t altitude;
    mavlink_msg_altitude_decode(&message, &altitude);

    //    qDebug() << "altitude: " << altitude.altitude_amsl;
}

void TelemetryHandler::processGlobalPosition(const mavlink_message_t& message)
{
    mavlink_global_position_int_t global_position;
    mavlink_msg_global_position_int_decode(&message, &global_position);

    m_pTree->appendProperties("MAV 23",
                              QJsonObject({ { "latitude",
                                              static_cast<float>(global_position.lat) / 10000000 },
                                            { "longitude",
                                              static_cast<float>(global_position.lon) / 10000000 },
                                            { "satelliteAltitude", global_position.alt / 1000 } }));
}
