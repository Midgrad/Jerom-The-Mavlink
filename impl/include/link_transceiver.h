#ifndef LINK_TRANSCEIVER_H
#define LINK_TRANSCEIVER_H

#include "i_link_transceiver.h"

#include "jerom_traits.h"

namespace md::data_source
{
class LinkTransceiver : public ILinkTransceiver
{
    Q_OBJECT

public:
    LinkTransceiver(const data_source::LinkPtr& link, QObject* parent = nullptr);

public slots:
    void start() override;
    void stop() override;

    void send(const QByteArray& data);

    //protected:
    //    void timerEvent(QTimerEvent* event) override;

signals:
    void receivedData(const QByteArray& data);

private:
    void receiveData();
    data_source::LinkPtr const m_link;
};
} // namespace md::data_source

#endif // LINK_TRANSCEIVER_H
