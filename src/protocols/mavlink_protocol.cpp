#include "mavlink_protocol.h"

#include <QDebug>
#include <QTimerEvent>

using namespace md::domain;

namespace
{
constexpr int interval = 100;
} // namespace

MavlinkProtocol::MavlinkProtocol(IMavlinkHandlerFactory* factory, QObject* parent) :
    ICommunicationProtocol(parent),
    m_handlers(factory->create(&m_context))
{
    for (IMavlinkHandler* handler : qAsConst(m_handlers))
    {
        handler->setParent(this);
        connect(handler, &IMavlinkHandler::sendMessage, this, &MavlinkProtocol::sendMessage);
    }
}

void MavlinkProtocol::start()
{
    for (auto thread : m_linkTransceiverThreaded)
    {
        thread->start();
    }
}

void MavlinkProtocol::stop()
{
    if (m_timerId)
    {
        this->killTimer(m_timerId);
        m_timerId = 0;
    }

    for (auto thread : m_linkTransceiverThreaded)
    {
        thread->stop();
    }

    emit finished();
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

    for (IMavlinkHandler* handler : m_handlers)
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
