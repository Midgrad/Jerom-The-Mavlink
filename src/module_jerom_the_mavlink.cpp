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

    // TODO:Create herder/configurator for managing loodsman with json
    // -------------------------------
    loodsman::link_type l_type;

    if (doc["type"].toString() == "udp")
        l_type = loodsman::link_type::udp;
    else if (doc["type"].toString() == "tcp")
        qFatal("Not implemented");
    else if (doc["type"].toString() == "serial")
        qFatal("Not implemented");
    else
        qFatal("Unknown link type! Aborting");

    WorkerDataReceive* udpworker = new WorkerDataReceive(l_type, doc["port"].toInt(), this);

    connect(udpworker, &WorkerDataReceive::result, this, &ModuleJeromTheMavlink::on_message);

    QThreadPool::globalInstance()->start(udpworker);
    // -------------------------------
}

void ModuleJeromTheMavlink::on_message(const std::string& data)
{
    //    qDebug() << data.c_str();
}

void ModuleJeromTheMavlink::done()
{
}
