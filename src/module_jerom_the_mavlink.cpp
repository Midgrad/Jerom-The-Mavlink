#include "module_jerom_the_mavlink.h"

//#include <QDebug>
#include <QFile>
//#include <QJsonArray>
#include <QJsonDocument>
//#include <QJsonObject>
//#include <QObject>
//#include <QThreadPool>

//#include "common/mavlink.h"
//#include "heartbeat_handler.h"
//#include "i_mavlink_handler.h"
//#include "telemetry_handler.h"
//#include "worker_data_receive.h"

#include "mavlink_transceiver.h"
#include "mavlink_transceiver_threaded.h"

//Q_DECLARE_METATYPE(std::string)

namespace
{
constexpr char path[] = "./link_config.json";
} // namespace

using namespace jerom_mavlink::app;
//using namespace jerom_mavlink::receive;

ModuleJeromTheMavlink::ModuleJeromTheMavlink()
{
}

void ModuleJeromTheMavlink::init()
{
    // TODO: to comman json config reader
    QFile file(::path);
    file.open(QIODevice::ReadOnly | QIODevice::Text);

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    file.close();

    auto transciever = new domain::MavlinkTransciever(doc.array());

    //    WorkerDataReceive* udpworker = new WorkerDataReceive(l_type, linkConfig["port"].toInt(), this);

    //    connect(udpworker, &WorkerDataReceive::result, this, &ModuleJeromTheMavlink::on_message);

    //    QThreadPool::globalInstance()->start(udpworker);
    //    // -------------------------------
}

//void ModuleJeromTheMavlink::on_message(const QByteArray& data)
//{
//    mavlink_message_t message;
//    mavlink_status_t status;

//    for (int pos = 0; pos < data.length(); ++pos)
//    {
//        if (!mavlink_parse_char(0, data[pos], &message, &status))
//            continue;
//    }

//    QVector<IMavlinkHandler*> m_handlers;

//    HeartbeatHandler hbt_hndlr;
//    TelemetryHandler tlmtr_hndlr;

//    m_handlers.append(&hbt_hndlr);
//    m_handlers.append(&tlmtr_hndlr);

//    for (IMavlinkHandler* handler : m_handlers)
//    {
//        if (handler->canParse(message.msgid))
//        {
//            handler->parseMessage(message);
//        }
//    }
//}

void ModuleJeromTheMavlink::done()
{
    // m_transciever->stop();
}
