#include "mavlink_transceiver.h"

#include <QDebug>
#include <QTimerEvent>

#include "link_transceiver.h"
#include "link_transceiver_threaded.h"

using namespace md::domain;

namespace
{
constexpr int interval = 100;
} // namespace

MavlinkTransceiver::MavlinkTransceiver(data_source::LinkService* configuration,
                                       IMavlinkHandlerFactory* factory, QObject* parent) :
    ICommunicationProtocol(parent),
    m_configuration(configuration),
    m_handlers(factory->create(&m_context))
{
    for (IMavlinkHandler* handler : qAsConst(m_handlers))
    {
        handler->setParent(this);
        connect(handler, &IMavlinkHandler::sendMessage, this, &MavlinkTransceiver::sendMessage);
    }

    for (const auto& link : m_configuration->links())
    {
        auto linkT = new data_source::LinkTransceiver(link, m_configuration->factory(), nullptr);
        auto linkTT = new data_source::LinkTransceiverThreaded(linkT, this);
        m_linkTransceiverThreaded.append(linkTT);

        QObject::connect(linkT, &data_source::LinkTransceiver::receivedData, this,
                         &domain::MavlinkTransceiver::receiveData);
        QObject::connect(this, &domain::MavlinkTransceiver::sendData, linkT,
                         &data_source::LinkTransceiver::send);
    }
}

void MavlinkTransceiver::start()
{
    for (auto thread : m_linkTransceiverThreaded)
    {
        thread->start();
    }
}

void MavlinkTransceiver::stop()
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

void MavlinkTransceiver::receiveData(const QByteArray& data)
{
    this->parseMessage(data);
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
        handler->parse(message);
    }
}

void MavlinkTransceiver::sendMessage(const mavlink_message_t& message)
{
    quint8 buffer[MAVLINK_MAX_PACKET_LEN];
    int lenght = mavlink_msg_to_send_buffer(buffer, &message);
    if (!lenght)
        return;

    QByteArray data((const char*) buffer, lenght);

    emit sendData(data);
}
