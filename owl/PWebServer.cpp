/* Owl: BLAST status GUI
 *
 * This file is part of Owl.
 *
 * Owl (originally "palantir") is copyright (C) 2002-2014 University of Toronto
 *
 * Owl is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Owl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Owl; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "PWebServer.h"
#include "PMainWindow.h"
#include <QHostAddress>
#include <QMessageBox>
#include <QJsonDocument>
#include <QJsonObject>

PWebServer::PWebServer(PMainWindow* parent, const int& port)
  : QObject(parent)
  , m_port(port)
  , m_server(new QWebSocketServer("owl", QWebSocketServer::NonSecureMode, this))
  , m_parent(parent)
{
    connect(m_server, SIGNAL(newConnection()), this, SLOT(handleNewConnection()));
    if (!m_server->listen(QHostAddress::Any, port)) {
        QMessageBox::warning(parent, "Bad port", "Could not listen on port " + QString::number(port) + ": " + m_server->errorString(),
          QMessageBox::Ok);
        delete m_server;
        m_server = 0;
    } else {
        //qDebug() << "Server is up on port" << port;
    }
}

PWebServer::~PWebServer() {
    // QWebSocketServer does not take ownership of QWebSockets, according to QWebSocketServer::nextPendingConnection
    for (int i = 0; i < m_sockets.length(); ++i) {
        delete m_sockets[i];
    }
}

void PWebServer::handleNewConnection() {
    while (m_server->hasPendingConnections()) {
        QWebSocket* socket = m_server->nextPendingConnection();
        writeState(socket);
        m_sockets.push_back(socket);
    }
}

void PWebServer::update() {
    for (int i = 0; i < m_sockets.length(); ++i) {
        writeUpdates(m_sockets[i]);
    }
}

void PWebServer::writeState(QWebSocket* socket) {
    if (socket->isValid()) {
        QJsonDocument document = QJsonDocument::fromVariant(m_parent->state());
        socket->sendTextMessage(document.toJson(QJsonDocument::Compact));
    }
}

void PWebServer::writeUpdates(QWebSocket* socket) {
    if (socket->isValid()) {
        QJsonDocument document = QJsonDocument::fromVariant(m_parent->stateChanges());
        socket->sendTextMessage(document.toJson(QJsonDocument::Compact));
    }
}
