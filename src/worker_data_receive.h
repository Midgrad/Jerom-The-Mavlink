#ifndef DATA_RECEIVE_H
#define DATA_RECEIVE_H

// clang-format off
#include <QObject>
#include <QRunnable>
#include <string>

namespace jerom_mavlink::receive
{
class WorkerDataReceive
    : public QObject
    , public QRunnable
{
    Q_OBJECT

public:
    explicit WorkerDataReceive(QObject* parent = nullptr);
    void run() override;
signals:
    void result(std::string);

protected:
    std::string m_data;
};

} // namespace jerom_mavlink::receive
#endif //JEROM_WORKER_DATA_RECEIVE_H
