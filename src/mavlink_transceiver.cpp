#include "mavlink_transceiver.h"

#include <QDebug>
#include <QTimerEvent>

using namespace md::domain;

namespace
{
constexpr int interval = 1;
} // namespace

MavlinkTransceiver::MavlinkTransceiver(const QMap<QString, linkPtr>& links,
                                       IMavlinkHandlerFactory* factory, QObject* parent) :
    IMavlinkTransceiver(parent),
    m_links(links),
    m_handlers(factory->create(&m_context))
{
    for (IMavlinkHandler* handler : qAsConst(m_handlers))
    {
        handler->setParent(this);
        connect(handler, &IMavlinkHandler::sendMessage, this, &MavlinkTransceiver::send);
    }
}

void MavlinkTransceiver::start()
{
    m_timerId = this->startTimer(::interval);
}

void MavlinkTransceiver::stop()
{
    if (m_timerId)
    {
        this->killTimer(m_timerId);
        m_timerId = 0;
    }

    emit finished();
}

void MavlinkTransceiver::timerEvent(QTimerEvent* event)
{
    if (event->timerId() != m_timerId)
        return QObject::timerEvent(event);

    this->receiveData();
}

void MavlinkTransceiver::receiveData()
{
    std::string received_data;
    for (linkPtr link : qAsConst(m_links))
    {
        // FIXME: unblocking read
        received_data = link->receive();
        this->parseMessage(QByteArray::fromStdString(received_data));
    }
}

void MavlinkTransceiver::parseMessage(const QByteArray& data)
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
        if (handler->canParse(message.msgid))
        {
            handler->parseMessage(message);
        }
    }
}

void MavlinkTransceiver::send(const mavlink_message_t& message)
{
    quint8 buffer[MAVLINK_MAX_PACKET_LEN];
    int lenght = mavlink_msg_to_send_buffer(buffer, &message);
    if (!lenght)
        return;

    QByteArray data((const char*) buffer, lenght);
    for (linkPtr link : qAsConst(m_links))
    {
        link->send(data.toStdString());
    }
}
