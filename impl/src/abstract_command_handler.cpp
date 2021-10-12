#include "abstract_command_handler.h"

using namespace md::domain;

AbstractCommandHandler::AbstractCommandHandler(MavlinkHandlerContext* context, QObject* parent) :
    IMavlinkHandler(context, parent)
{
}

// TODO: CommandService
void AbstractCommandHandler::sendCommandLong(quint8 mavId, quint16 commandId,
                                             const QVariantList& args, int attempt)
{
    mavlink_message_t message;
    mavlink_command_long_t mavCommand;

    mavCommand.target_system = mavId;
    mavCommand.target_component = 0;
    mavCommand.confirmation = attempt;
    mavCommand.command = commandId;

    if (args.count() > 0)
        mavCommand.param1 = args.at(0).toFloat();
    if (args.count() > 1)
        mavCommand.param2 = args.at(1).toFloat();
    if (args.count() > 2)
        mavCommand.param3 = args.at(2).toFloat();
    if (args.count() > 3)
        mavCommand.param4 = args.at(3).toFloat();
    if (args.count() > 4)
        mavCommand.param5 = args.at(4).toFloat();
    if (args.count() > 5)
        mavCommand.param6 = args.at(5).toFloat();
    if (args.count() > 6)
        mavCommand.param7 = args.at(6).toFloat();

    mavlink_msg_command_long_encode_chan(m_context->systemId, m_context->compId, 0, &message,
                                         &mavCommand); // TODO: link channel
    emit sendMessage(message);
}

void AbstractCommandHandler::subscribeCommand(const QString& command, CommandSendCallback callback)
{
    connect(m_context->pTree, &IPropertyTree::propertiesChanged, this,
            [this, command, callback](const QString& node, const QVariantMap& properties) {
                if (properties.contains(command))
                {
                    this->m_context->pTree->removeProperties(node, { command });
                    callback(node, properties.value(command));
                }
            });
}
