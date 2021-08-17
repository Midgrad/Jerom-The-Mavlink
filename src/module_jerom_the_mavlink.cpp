#include "module_jerom_the_mavlink.h"

#include "common/mavlink.h"
#include "worker_data_receive.h"
#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QObject>
#include <QThreadPool>

//Q_DECLARE_METATYPE(std::string)

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
    //    qRegisterMetaType<std::string>("std::string");

    QFile file(::path);
    file.open(QIODevice::ReadOnly | QIODevice::Text);

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    file.close();

    QJsonObject linkConfig = doc.object();

    // TODO:Create herder/configurator for managing loodsman with json
    // -------------------------------
    loodsman::link_type l_type;
    if (linkConfig["type"].toString() == "udp")
        l_type = loodsman::link_type::udp;
    else if (linkConfig["type"].toString() == "tcp")
        qFatal("Not implemented");
    else if (linkConfig["type"].toString() == "serial")
        qFatal("Not implemented");
    else
        qFatal("Unknown link type! Aborting");

    WorkerDataReceive* udpworker = new WorkerDataReceive(l_type, linkConfig["port"].toInt(), this);

    connect(udpworker, &WorkerDataReceive::result, this, &ModuleJeromTheMavlink::on_message);

    QThreadPool::globalInstance()->start(udpworker);
    // -------------------------------
}

void ModuleJeromTheMavlink::on_message(const QByteArray& data)
{
    mavlink_message_t message;
    mavlink_status_t status;
    mavlink_heartbeat_t heartbeat;

    for (int pos = 0; pos < data.length(); ++pos)
    {
        if (!mavlink_parse_char(0, data[pos], &message, &status))
            continue;
    }

    if (MAVLINK_MSG_ID_HEARTBEAT == message.msgid)
    {
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        qDebug() << heartbeat.type;
        qDebug() << heartbeat.base_mode;
        qDebug() << heartbeat.system_status;
    }

    //    qDebug() << data;
}

void ModuleJeromTheMavlink::done()
{
}
