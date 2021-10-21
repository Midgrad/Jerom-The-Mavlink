#include "link_transceiver.h"

#include <QDebug>

using namespace md::data_source;

LinkTransceiver::LinkTransceiver(const data_source::LinkPtr& link, QObject* parent) :
    ILinkTransceiver(parent),
    m_link(link)
{
}

void LinkTransceiver::start()
{
    // FIXME: for testing purposes only
    while (true)
    {
        receiveData();
    }
}

void LinkTransceiver::stop()
{
    emit finished();
}

void LinkTransceiver::receiveData()
{
    std::string received_data;
    // FIXME: unblocking read
    received_data = m_link->receive();

    //    qDebug() << "Emitting received data";
    emit receivedData(QByteArray::fromStdString(received_data));
}

void LinkTransceiver::send(const QByteArray& data)
{
    //    qDebug() << "Sending data";
    m_link->send(data.toStdString());
}
