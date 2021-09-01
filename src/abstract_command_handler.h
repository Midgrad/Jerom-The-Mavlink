#ifndef ABSTRACT_COMMAND_HANDLER_H
#define ABSTRACT_COMMAND_HANDLER_H

#include "i_mavlink_handler.h"

namespace md::domain
{
using CommandSendCallback = std::function<void(const QString&, const QVariant&)>;

class AbstractCommandHandler : public IMavlinkHandler
{
    Q_OBJECT

public:
    AbstractCommandHandler(MavlinkHandlerContext* context, QObject* parent);

    void sendCommandLong(quint8 mavId, quint16 commandId, const QVariantList& args, int attempt);

protected:
    void subscribeCommand(const QString& command, CommandSendCallback callback);
};

} // namespace md::domain

#endif // ABSTRACT_COMMAND_HANDLER_H
