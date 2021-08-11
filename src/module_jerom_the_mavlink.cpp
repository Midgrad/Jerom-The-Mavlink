#include "module_jerom_the_mavlink.h"

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


}

void ModuleJeromTheMavlink::done()
{
}
