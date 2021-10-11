#ifndef MAVLINK_TRANSCIEVER_THREADED_H
#define MAVLINK_TRANSCIEVER_THREADED_H

#include "i_mavlink_transceiver.h"

#include <QThread>

namespace md::domain
{
class MavlinkTranscieverThreaded : public IMavlinkTransceiver
{
    Q_OBJECT

public:
    MavlinkTranscieverThreaded(IMavlinkTransceiver* worker, QObject* parent = nullptr);
    ~MavlinkTranscieverThreaded() override;

public slots:
    void start() override;
    void stop() override;

private:
    IMavlinkTransceiver* const m_worker;
    QThread* const m_thread;
};
} // namespace md::domain

#endif // MAVLINK_TRANSCIEVER_THREADED_H
