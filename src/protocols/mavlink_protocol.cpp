#include "mavlink_protocol.h"

#include <QDebug>
#include <QTimerEvent>

using namespace md::data_source;

namespace
{
constexpr int interval = 100;
} // namespace

MavlinkProtocol::MavlinkProtocol(domain::IMavlinkHandlerFactory* factory, QObject* parent) :
    ICommunicationProtocol(parent),
    m_handlers(factory->create(&m_context))
{
    for (domain::IMavlinkHandler* handler : qAsConst(m_handlers))
    {
        handler->setParent(this);
        connect(handler, &domain::IMavlinkHandler::sendMessage, this, &MavlinkProtocol::sendMessage);
    }
}

void MavlinkProtocol::receiveData(const QByteArray& data)
{
    this->parseMessage(data);
}

void MavlinkProtocol::parseMessage(const QByteArray& data)
{
    mavlink_message_t message;
    mavlink_status_t status;

    for (int pos = 0; pos < data.length(); ++pos)
    {
        if (!mavlink_parse_char(0, data[pos], &message, &status))
            continue;
    }

    for (domain::IMavlinkHandler* handler : m_handlers)
    {
        handler->parse(message);
    }
}

void MavlinkProtocol::sendMessage(const mavlink_message_t& message)
{
    quint8 buffer[MAVLINK_MAX_PACKET_LEN];
    int lenght = mavlink_msg_to_send_buffer(buffer, &message);
    if (!lenght)
        return;

    QByteArray data((const char*) buffer, lenght);

    emit sendData(data);
}
