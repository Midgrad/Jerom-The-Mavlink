#include "module_jerom_the_mavlink.h"

#include "link_factory.h"
#include "worker_data_receive.h"
#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

namespace
{
constexpr char path[] = "./link_config.json";
} // namespace

using namespace jerom_mavlink::app;
using namespace loodsman;

ModuleJeromTheMavlink::ModuleJeromTheMavlink()
{
}

void ModuleJeromTheMavlink::init()
{
    QFile file(::path);
    file.open(QIODevice::ReadOnly | QIODevice::Text);

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    file.close();

    //    qDebug() << doc["port"].toInt();
    //    qDebug() << doc["type"].toString();

    link_ptr link_listen;
    link_type l_type;

    if (doc["type"].toString() == "udp")
        l_type = link_type::udp;

    int result = factory(link_listen, l_type, doc["port"].toInt());
    Q_ASSERT(result == 0);

    //    QThreadPool::globalInstance()->start(calc);

    //    std::string received_data(link_listen->receive());
    //
    //    qDebug() << received_data.c_str();
}

void ModuleJeromTheMavlink::done()
{
}
