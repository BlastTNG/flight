#include "server.h"

// Starts a tcp server that sends config data to any client that connects
Server::Server(QObject * parent) :
    QObject(parent)
{
    // set up the server
    tcpserver = new QTcpServer();
    connect(tcpserver, SIGNAL(newConnection()), this, SLOT(sendServerData()));

    if (!tcpserver->listen(QHostAddress::Any, SLAVEPORT)) {
        qDebug() << "Server could not start on port " << QString::number(SLAVEPORT);
    }
}

Server::~Server() {
    if (tcpserver) delete tcpserver;
}

void Server::sendServerData() {
    // need to grab the socket
    QTcpSocket *socket = tcpserver->nextPendingConnection();
    if (!socket) return;

    // socket->write();

    socket->flush();
    socket->waitForBytesWritten(3000);

    socket->close();
}

/*
 * Connects to a server guaca and slaves to it by retrieving
 * the configuration file and applying those settings.
 * Configuration includes current state of mole options
 * as well as whether or not mole clients have been
 * initialized.
 */

bool Server::getServerData(QString host)
{
    return true;
}
