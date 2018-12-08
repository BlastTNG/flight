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

#include <QDebug>
#include "PServer.h"

PServer::PServer(QString ckey) : _sema("_owlSema"+ckey,1,QSystemSemaphore::Create), _smemHtml("_owlHtml"+ckey),
    _smemCSS("_owlCSS"+ckey),_smemLayout("_owlLayout"+ckey),_smemData("_owlData"+ckey), key(ckey)
{
    _sema.acquire();
}

PServer::~PServer() {
    _sema.release();
    if(_smemHtml.isAttached()) _smemHtml.detach();
    if(_smemCSS.isAttached()) _smemCSS.detach();
    if(_smemLayout.isAttached()) _smemLayout.detach();
    if(_smemData.isAttached()) _smemData.detach();
}

void PServer::clockOn(const QString& html,const QString&css,const QString&layout,const QString&data) {
  qDebug() << "pserver clockon";
    if(_smemHtml.isAttached()) _smemHtml.detach();
    if(_smemCSS.isAttached()) _smemCSS.detach();
    if(_smemLayout.isAttached()) _smemLayout.detach();
    if(_smemData.isAttached()) _smemData.detach();

    _smemHtml.create(html.size()+1);
    _smemCSS.create(css.size()+1);
    _smemLayout.create(layout.size()+1);
    _smemData.create(data.size()+1);

    _smemHtml.lock();
    _smemCSS.lock();
    _smemLayout.lock();
    _smemData.lock();

    memcpy(_smemHtml.data(),html.toLatin1().data(),html.size()+1);
    memcpy(_smemCSS.data(),css.toLatin1().data(),css.size()+1);
    memcpy(_smemLayout.data(),layout.toLatin1().data(),layout.size()+1);
    memcpy(_smemData.data(),data.toLatin1().data(),data.size()+1);

    _smemHtml.unlock();
    _smemCSS.unlock();
    _smemLayout.unlock();
    _smemData.unlock();

    _sema.release();
    QTimer::singleShot(30,this,SLOT(clockOff()));
}

void PServer::clockOff() {
    _sema.acquire();
    _smemHtml.detach();
    _smemCSS.detach();
    _smemLayout.detach();
    _smemData.detach();
}
