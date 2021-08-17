#ifndef JEROM_WORKER_DATA_RECEIVE_H
#define JEROM_WORKER_DATA_RECEIVE_H

#include "link_factory.h"
#include <string>
#include <QObject>
#include <QRunnable>

namespace jerom_mavlink::receive
{
class WorkerDataReceive
    : public QObject
    , public QRunnable
{
    Q_OBJECT

public:
    explicit WorkerDataReceive(loodsman::link_type l_type, int port, QObject* parent = nullptr);
    void run() override;
signals:
    void result(const std::string& data);

protected:
    std::string m_data;
    loodsman::link_ptr m_link;
};

} // namespace jerom_mavlink::receive
#endif //JEROM_WORKER_DATA_RECEIVE_H
