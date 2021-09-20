#include "mavlink_transceiver.h"

#include <string>
#include <QDebug>
#include <QTimerEvent>

using namespace md::domain;

namespace
{
constexpr int interval = 1;
} // namespace

MavlinkTransceiver::MavlinkTransceiver(data_source::LinkConfiguration* configuration,
                                       IMavlinkHandlerFactory* factory, QObject* parent) :
    IMavlinkTransceiver(parent),
    m_links(configuration->createLinks()),
    m_handlers(factory->create(&m_context)),
    m_configuration(configuration)
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
    m_configuration->checkHandlers();

    for (const data_source::LinkPtr& link : qAsConst(m_links))
    {
        link->asyncReceive(std::bind(parseMessage, std::placeholders::_1, this));
    }
}

void MavlinkTransceiver::parseMessage(const std::string& data, MavlinkTransceiver* transceiver)
{
    QByteArray bytearray = QByteArray::fromStdString(data);

    mavlink_message_t message;
    mavlink_status_t status;

    for (int pos = 0; pos < bytearray.length(); ++pos)
    {
        if (!mavlink_parse_char(0, data[pos], &message, &status))
            continue;
    }

    for (IMavlinkHandler* handler : transceiver->handlers())
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
    for (const data_source::LinkPtr& link : qAsConst(m_links))
    {
        link->send(data.toStdString());
    }
}
const QVector<IMavlinkHandler*>& MavlinkTransceiver::handlers() const
{
    return m_handlers;
}
