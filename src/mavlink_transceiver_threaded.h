#ifndef MAVLINK_TRANSCIEVER_THREADED_H
#define MAVLINK_TRANSCIEVER_THREADED_H

#include "i_mavlink_transceiver.h"

#include <QThread>

namespace md::domain
{
class MavlinkTranscieverThreaded : public IMavlinkTransciever
{
    Q_OBJECT

public:
    MavlinkTranscieverThreaded(IMavlinkTransciever* worker, QObject* parent = nullptr);
    ~MavlinkTranscieverThreaded() override;

public slots:
    void start() override;
    void stop() override;

private:
    IMavlinkTransciever* const m_worker;
    QThread* const m_thread;
};
} // namespace jerom_mavlink::domain

#endif // MAVLINK_TRANSCIEVER_THREADED_H
