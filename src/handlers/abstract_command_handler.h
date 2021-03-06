#ifndef ABSTRACT_COMMAND_HANDLER_H
#define ABSTRACT_COMMAND_HANDLER_H

#include "i_mavlink_handler.h"

namespace md::domain
{
class AbstractCommandHandler : public IMavlinkHandler
{
    Q_OBJECT

public:
    AbstractCommandHandler(MavlinkHandlerContext* context, QObject* parent);

    void sendCommandLong(quint8 mavId, quint16 commandId, const QVariantList& args, int attempt);
};

} // namespace md::domain

#endif // ABSTRACT_COMMAND_HANDLER_H
