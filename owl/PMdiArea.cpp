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

#include "PMdiArea.h"
#include "PMainWindow.h"

void PMdiArea::dragEnterEvent(QDragEnterEvent *ev)
{
    if(ev->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist"))
    {
        QByteArray encoded = ev->mimeData()->data("application/x-qabstractitemmodeldatalist");
        QDataStream stream(&encoded, QIODevice::ReadOnly);

        int row, col;
        QMap<int,  QVariant> roleDataMap;
        stream >> row >> col >> roleDataMap;
        if((!row||row==6)&&!col) {    //HACKHACKHACKHACK!!!
            ev->acceptProposedAction();
        }
    }
}

void PMdiArea::dropEvent(QDropEvent *ev)
{
    if(ev->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist"))
    {
        QByteArray encoded = ev->mimeData()->data("application/x-qabstractitemmodeldatalist");
        QDataStream stream(&encoded, QIODevice::ReadOnly);

        int row, col;
        QMap<int,  QVariant> roleDataMap;
        stream >> row >> col >> roleDataMap;
        if(!row&&!col) {    //HACKHACKHACKHACK!!!
            createPBox(ev->pos().x(),ev->pos().y());
        } else if(row==6&&!col) {   //HACKHACKHACKHACK!!!
            createOwl(ev->pos().x(),ev->pos().y());
        }
    }
}

void PMdiArea::createPBox(int x,int y,PBox*c_pbox)
{
    PBox* pbox=c_pbox?c_pbox:new PBox("New Box", font());
    pbox->setParent(this);

    if (c_pbox) {
      pbox->setGeometry(c_pbox->geometry().x()/_H*_H,c_pbox->geometry().y()/_H*_H,
                        c_pbox->geometry().width()/_H*_H,c_pbox->geometry().height()/_H*_H);
    } else {
        pbox->setGeometry(x/_H*_H,y/_H*_H,pbox->width()/_H*_H,pbox->height()/_H*_H);
        pbox->adjustSize();
    }
    dynamic_cast<PMainWindow*>(PMainWindow::me)->_pboxList.push_back(pbox);
    pbox->show();
    emit newBox(pbox);
}

void PMdiArea::createOwl(int x,int y,POwlAnimation*c_owl)
{
    POwlAnimation* owl=c_owl?c_owl:new POwlAnimation(_H);
    owl->setParent(this);
    if(!c_owl) {
        owl->setGeometry(x/_H*_H,y/_H*_H,owl->sizeHint().width()/_H*_H,owl->sizeHint().height()/_H*_H);
    }
    dynamic_cast<PMainWindow*>(PMainWindow::me)->_owlList.push_back(owl);
    owl->show();
    emit newOwl(owl);
}
