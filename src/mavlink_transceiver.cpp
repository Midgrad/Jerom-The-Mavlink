#include "mavlink_transceiver.h"

#include <QDebug>
#include <QTimerEvent>

#include "link_factory.h"

using namespace jerom_mavlink::domain;

namespace
{
constexpr int interval = 1;
} // namespace

MavlinkTransciever::MavlinkTransciever(const QMap<QString, loodsman::LinkPtr>& links,
                                       IMavlinkHandlerFactory* factory, QObject* parent) :
    IMavlinkTransciever(parent),
    m_links(links),
    m_handlers(factory->create())
{
    for (IMavlinkHandler* handler : qAsConst(m_handlers))
    {
        handler->setParent(this);
    }
}

void MavlinkTransciever::start()
{
    m_timerId = this->startTimer(::interval);
}

void MavlinkTransciever::stop()
{
    if (m_timerId)
    {
        this->killTimer(m_timerId);
        m_timerId = 0;
    }

    emit finished();
}

void MavlinkTransciever::timerEvent(QTimerEvent* event)
{
    if (event->timerId() != m_timerId)
        return QObject::timerEvent(event);

    this->receiveData();
}

void MavlinkTransciever::receiveData()
{
    std::string received_data;
    for (const loodsman::LinkPtr& link : qAsConst(m_links))
    {
        // FIXME: unblocking read
        received_data = link->receive();
        this->parseMessage(QByteArray::fromStdString(received_data));
    }
}

void MavlinkTransciever::parseMessage(const QByteArray& data)
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

void MavlinkTransciever::send(const QByteArray& data)
{
    for (const loodsman::LinkPtr& link : qAsConst(m_links))
    {
        link->send(data.toStdString());
    }
}
