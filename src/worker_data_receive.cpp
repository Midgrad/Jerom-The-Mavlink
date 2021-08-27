#include "worker_data_receive.h"
#include "link_factory.h"
#include <QDebug>

using namespace jerom_mavlink::domain;
using namespace loodsman;

WorkerDataReceive::WorkerDataReceive(link_type l_type, int port, QObject* parent) : QObject(parent)
{
    int result = factory(m_link, l_type, port);
    Q_ASSERT(result == 0);
}

void WorkerDataReceive::run()
{
    while (true)
    {
        std::string received_data(m_link->receive());
        emit result(QByteArray::fromStdString(received_data));
        //        qDebug() << "Data received";
    }
}
