#include "worker_data_receive.h"
#include <QDebug>

using namespace jerom_mavlink::receive;

WorkerDataReceive::WorkerDataReceive(QObject* parent) : QObject(parent)
{
}

void WorkerDataReceive::run()
{
    //    std::string received_data(link_listen->receive());
    //    qDebug() << received_data.c_str();
    //
    //    emit result(received_data);
}
