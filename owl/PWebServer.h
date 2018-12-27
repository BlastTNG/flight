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

#ifndef PWEBSERVER_H
#define PWEBSERVER_H

#include <QList>

#include <QtWebSockets/QWebSocket>
#include <QtWebSockets/QWebSocketServer>

#include <QObject>

class PMainWindow;

class PWebServer : public QObject
{
  Q_OBJECT
  public:
    PWebServer(PMainWindow* parent, const int& port);
    virtual ~PWebServer();
    Q_SLOT void handleNewConnection();
    Q_SLOT void update();
  private:
    void writeState(QWebSocket*);
    void writeUpdates(QWebSocket*);
    int m_port;
    QWebSocketServer* m_server;
    QList<QWebSocket*> m_sockets;
    PMainWindow* m_parent;
};

#endif // PWEBSERVER_H
