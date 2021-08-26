#ifndef I_MAVLINK_TRANSCIEVER_H
#define I_MAVLINK_TRANSCIEVER_H

#include <QObject>

namespace jerom_mavlink::domain
{
class IMavlinkTransciever : public QObject
{
    Q_OBJECT

public:
    IMavlinkTransciever(QObject* parent) : QObject(parent)
    {
    }
    virtual ~IMavlinkTransciever() = default;

public slots:
    virtual void start() = 0;
    virtual void stop() = 0;
};
} // namespace jerom_mavlink::domain

#endif // I_MAVLINK_TRANSCIEVER_H
