#ifndef I_MAVLINK_TRANSCEIVER_H
#define I_MAVLINK_TRANSCEIVER_H

#include <QObject>

namespace md::domain
{
class IMavlinkTransceiver : public QObject
{
    Q_OBJECT

public:
    IMavlinkTransceiver(QObject* parent) : QObject(parent)
    {
    }
    virtual ~IMavlinkTransceiver() = default;

public slots:
    virtual void start() = 0;
    virtual void stop() = 0;

signals:
    void finished();
};
} // namespace md::domain

#endif // I_MAVLINK_TRANSCEIVER_H
