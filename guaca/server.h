#ifndef SERVER_H
#define SERVER_H

#include <QObject>
#include <QWidget>
#include <QTcpSocket>
#include <QTcpServer>

#define SLAVEPORT 40004

class Server : public QObject
{
    Q_OBJECT

public:
    explicit Server(QObject *parent = nullptr);
    ~Server();

    bool getServerData(QString host);

public slots:
    void sendServerData();

private:
    QTcpServer *tcpserver;
};

#pragma pack(push, 1)
struct SharedData {
    uint8_t mole_active;
};
#pragma pack(pop)


#endif // SERVER_H
