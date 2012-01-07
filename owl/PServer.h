/* Owl: BLAST status GUI
 *
 * This file is part of Owl.
 *
 * Owl (originally "palantir") is copyright (C) 2002-2012 University of Toronto
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

#include <QSystemSemaphore>
#include <QSharedMemory>
#include <QString>
#include <QObject>
#include <QTimer>

#ifndef PSERVER_H
#define PSERVER_H

class PServer : public QObject
{
    Q_OBJECT
    QSystemSemaphore _sema;
    QSharedMemory _smemHtml;
    QSharedMemory _smemCSS;
    QSharedMemory _smemLayout;
    QSharedMemory _smemData;

public:
    friend class QTimer;

    QString key;

    /*WARNING: this is __NOT__ platform independant. See the Qt documentation for the constructor for QSystemSemaphore*/
    PServer(QString ckey);

    ~PServer();

    void clockOn(const QString& html,const QString&css,const QString&layout,const QString&data);

public slots:
    void clockOff();
};

#endif // PSERVER_H
