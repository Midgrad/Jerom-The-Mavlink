#include "module_jerom_the_mavlink.h"

#include "worker_data_receive.h"
#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QObject>
#include <QThreadPool>

Q_DECLARE_METATYPE(std::string)

namespace
{
constexpr char path[] = "./link_config.json";
} // namespace

using namespace jerom_mavlink::app;
using namespace jerom_mavlink::receive;

ModuleJeromTheMavlink::ModuleJeromTheMavlink()
{
}

void ModuleJeromTheMavlink::init()
{

    qRegisterMetaType<std::string>("std::string");

    QFile file(::path);
    file.open(QIODevice::ReadOnly | QIODevice::Text);

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    file.close();

    // TODO: Split json and pass them into constructors

    WorkerDataReceive* udpworker = new WorkerDataReceive(loodsman::link_type::udp,
                                                         doc["port"].toInt(), this);

    connect(udpworker, &WorkerDataReceive::result, this, &ModuleJeromTheMavlink::on_message);

    QThreadPool::globalInstance()->start(udpworker);

    //    if (doc["type"].toString() == "udp")
    //        doc["port"].toInt()
}

void ModuleJeromTheMavlink::on_message(const std::string& data)
{
    qDebug() << data.c_str();
}

void ModuleJeromTheMavlink::done()
{
}
