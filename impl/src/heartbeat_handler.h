#ifndef HEARTBEAT_HANDLER_H
#define HEARTBEAT_HANDLER_H

#include "abstract_command_handler.h"
#include "i_command_service.h"
#include "i_mode_helper.h"
#include "i_vehicles_repository.h"

#include <QBasicTimer>
#include <QMap>

namespace md::domain
{
class HeartbeatHandler : public AbstractCommandHandler
{
    Q_OBJECT

public:
    HeartbeatHandler(MavlinkHandlerContext* context, IVehiclesRepository* vehiclesRepository,
                     ICommandsService* commandsService, QObject* parent = nullptr);
    ~HeartbeatHandler() override;

    bool canParse(quint32 msgId) override;
    void parseMessage(const mavlink_message_t& message) override;

    void sendMode(const QVariant& vehicleId, const QString& mode);
    void sendArm(const QVariant& vehicleId, bool arm);

    void processHeartbeat(const mavlink_message_t& message);

signals:
    void vehicleObtained(Vehicle* vehicle);

protected:
    void timerEvent(QTimerEvent* event) override;

private:
    IVehiclesRepository* const m_vehiclesRepository;
    QMap<quint8, QSharedPointer<data_source::IModeHelper>> m_modeHelpers;
    QMap<quint8, quint8> m_baseModes;
    QMap<Vehicle*, QBasicTimer*> m_vehicleTimers;
};
} // namespace md::domain

#endif // HEARTBEAT_HANDLER_H
