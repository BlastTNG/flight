/* Owl: BLAST status GUI
 *
 * This file is part of Owl.
 *
 * Owl (originally "palantir") is copyright (C) 2002-2011 University of Toronto
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

#include "POwlAnimation.h"
#include "PMainWindow.h"
#include "PMdiArea.h"

#include <QMainWindow>

Owl::Owl() : _lastNFrames(0),_label(new QLabel), _stage(0)
{
    _moveThingy=QPoint(-1,-1);
    for(int i=0;i<4;i++) {
        _pixMaps[i]=new QPixmap(":/icons/owl"+QString::number(i)+".png");
    }
    _pixMaps[4]=new QPixmap(":/icons/owl-1.png");

    setLayout(new QHBoxLayout);
    layout()->addWidget(_label);
    layout()->setAlignment(_label,Qt::AlignCenter);
}

Owl::~Owl()
{
    for(int i=0;i<5;i++) {
        delete _pixMaps[i];
    }
    PMainWindow* np=PMainWindow::me;
    np->_owlList.removeOne(this);
}

void Owl::gdUpdate(GetData::Dirfile* dirFile,int lastNFrames)
{
    if(lastNFrames!=_lastNFrames) {
        qsrand(dirFile->NFrames());
        _stage=qrand()%4;
        _label->setPixmap(*_pixMaps[_stage]);
        _lastNFrames=lastNFrames;
    } else {
        _stage=-1;
        _label->setPixmap(*_pixMaps[4]);
    }
}

QSize Owl::sizeHint()
{
    return QSize(100,200);
}

const int& Owl::stage() const
{
    return _stage;
}


void Owl::mouseMoveEvent(QMouseEvent *ev)
{
    if(ev->buttons()&Qt::LeftButton&&_moveThingy!=QPoint(-1,-1)) {
        QPoint p1=parentWidget()->mapFromGlobal(ev->globalPos());   //setGeometry refers to parent's (x,y)
        setGeometry((p1.x()-_moveThingy.x())/20*20,(p1.y()-_moveThingy.y())/20*20,
                    geometry().width()/20*20,geometry().height()/20*20);
    }
}

