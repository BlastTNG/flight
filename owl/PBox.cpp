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

#include "PBox.h"
#include "PMainWindow.h"
#include "PTimeDataItem.h"
#include "PDirfileDataItem.h"
#include "PBoxTitle.h"

PBox::PBox(QString boxTitle, QWidget*p) : QFrame(p), _pstyle(PStyle::noStyle), _layout(new QVBoxLayout()),
    _boxTitle(boxTitle), _geoMode(-1), _dirty(1)
{
    bool ok=0;
    for(int i=0;i<PStyle::_u.size();i++) {
        if(PStyle::_u[i]->name()=="defbox") {
            ok=1;
            _pstyle=PStyle::_u[i];
            break;
        }
    }

    _pstyle=ok?_pstyle:new PStyle("defbox",1,0,"white","blue",1);

    setFrameStyle(QFrame::Panel | QFrame::Plain);
    setLineWidth(1);
    setContentsMargins(2,2,2,2);
    setMidLineWidth(1);
    setFrameShadow(QFrame::Plain);
    _pbt=new PBoxTitle;
    _pbt->setText("[change me]");
    addTitle(_pbt);
    connect(PStyleNotifier::me,SIGNAL(change()),this,SLOT(styleLogic()));
    setWindowTitle(boxTitle);
    setLayout(_layout);
    _layout->setSpacing(0);
    setMinimumSize(100,50);
    setAcceptDrops(1);
    _layout->setContentsMargins(4,4,4,4);
    _layout->addSpacerItem(new QSpacerItem(0,0,QSizePolicy::Minimum,QSizePolicy::MinimumExpanding));

    QRect geo=geometry();
    geo.setX(((int)geo.x()/20)*20);
    geo.setY(((int)geo.y()/20)*20);
    geo.setWidth(((int)geo.width()/20)*20);
    geo.setHeight(((int)geo.height()/20)*20);
    if(geo!=geometry()) {
        setGeometry(geo);
    }
    setAutoFillBackground(1);
}

PBox::~PBox()
{
    PMainWindow::me->obviate(this);
}

void PBox::checkActivationState() {
    if(!children().contains(PMainWindow::me->currentObject())) {
        emit activated();
    }
}

void PBox::styleLogic() {
    if(_pstyle!=_lastPStyle|| //the reason this works is because PStyleChooser works via reference to ptr
            _pstyle->_dirty) {
        applyStyle(this,_pstyle);
        _lastPStyle=_pstyle;
    }
    setAutoFillBackground(1);
}

void PBox::setBoxTitle(const QString& boxTitle,bool force)
{
    if(!isCurrentObject()&&!force) return;

    if(_boxTitle==boxTitle) return; _boxTitle=boxTitle; _pbt->setText(boxTitle); emit textChanged(boxTitle);
}

void PBox::gdUpdate(GetData::Dirfile* dirFile,int lastNFrames)
{
    for(int i=0;i<_dataItems.size();i++) {
        _dataItems[i]->gdUpdate(dirFile,lastNFrames);
    }
    if(_dirty) {
        int minCapWidth=0;
        for(int i=0;i<_dataItems.size();i++) {
            _dataItems[i]->gdUpdate(dirFile,lastNFrames);
            int x=_dataItems[i]->_caption->fontMetrics().size(0,_dataItems[i]->_caption->text()).width();
            minCapWidth=qMax(minCapWidth,x);
            _dataItems[i]->_caption->setFixedHeight(12);
            _dataItems[i]->_data->setFixedHeight(12);
        }
        for(int i=0;i<_dataItems.size();i++) {
            if(_dataItems[i]->_caption->width()!=minCapWidth) {
                _dataItems[i]->_caption->setFixedWidth(minCapWidth);
            }
        }
        _dirty=0;
    }
}

void PBox::dragEnterEvent(QDragEnterEvent *ev)
{
    if(ev->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist"))
    {
        QByteArray encoded = ev->mimeData()->data("application/x-qabstractitemmodeldatalist");
        QDataStream stream(&encoded, QIODevice::ReadOnly);

        int row, col;
        QMap<int,  QVariant> roleDataMap;
        stream >> row >> col >> roleDataMap;
        if(row&&row<5&&!col) {    //HACKHACKHACKHACK!!!
            ev->acceptProposedAction();
        }
    }
}

void PBox::dropEvent(QDropEvent *ev)
{
    if(ev->mimeData()->hasFormat("application/x-qabstractitemmodeldatalist"))
    {
        QByteArray encoded = ev->mimeData()->data("application/x-qabstractitemmodeldatalist");
        QDataStream stream(&encoded, QIODevice::ReadOnly);

        int row, col;
        QMap<int,  QVariant> roleDataMap;
        stream >> row >> col >> roleDataMap;
        if(row&&row<5&&!col) {    //HACKHACKHACKHACK!!!
            switch(row) {
            case 1:
                addProperty("Number");
                break;
            case 2:
                addProperty("Multi");
                break;
            case 3:
                addProperty("Time/Date");
                break;
            case 4:
                addProperty("Dirfile");
                break;
            default:
                addProperty("Item");
                break;
            }

        }
    }
}

void PBox::addProperty(QString property)
{
    if(property=="Number") {
        addProperty(new PNumberDataItem(this, property));
    } else if(property=="Multi") {
        addProperty(new PMultiDataItem(this, property));
    } else if(property=="Time/Date") {
        addProperty(new PTimeDataItem(this, property));
    } else if(property=="Dirfile") {
        addProperty(new PDirfileDataItem(this, property));
    } else {
        addProperty(new PAbstractDataItem(this, property));
    }
}

void PBox::addProperty(PAbstractDataItem* padi)
{
    _dataItems.push_back(padi);
    _layout->insertWidget(_dataItems.size(),_dataItems.back()); //not -1 because of title
    _dataItems.back()->setFixedHeight(17);
    _dataItems.back()->show();
    emit newChild(_dataItems.back());
}

void PBox::addTitle(PBoxTitle* pbt)
{
    _layout->insertWidget(0,pbt);
    pbt->setFixedHeight(17);
    pbt->show();
}

void PBox::mousePressEvent(QMouseEvent *)
{
    _geoMode=-1;
    if(parentWidget()->children().indexOf(this)!=
            parentWidget()->children().size()-1)
    {
        QWidget*p =parentWidget();
        setParent(0);
        setParent(p);
        show();
        setFocus();
    }
    emit activated();
}

void PBox::mouseReleaseEvent(QMouseEvent *)
{
    _geoMode=-1;
}

void PBox::mouseMoveEvent(QMouseEvent *ev)
{
    if(ev->buttons()&Qt::LeftButton) {
        QPoint p1=parentWidget()->mapFromGlobal(ev->globalPos());   //setGeometry refers to parent's (x,y)
        QRect geo=geometry();
        if(_geoMode==-1) {
            int mdis=99999999;
            int mcase=-1;

            int dis2TL=(p1-geometry().topLeft()).manhattanLength();
            if(dis2TL<mdis) { mdis=dis2TL; mcase=0; }

            int dis2TR=(p1-geometry().topRight()).manhattanLength();
            if(dis2TR<mdis) { mdis=dis2TR; mcase=1; }

            int dis2BL=(p1-geometry().bottomLeft()).manhattanLength();
            if(dis2BL<mdis) { mdis=dis2BL; mcase=2; }

            int dis2BR=(p1-geometry().bottomRight()).manhattanLength();
            if(dis2BR<mdis) { mdis=dis2BR; mcase=3; }

            int dis2TC=(p1-QPoint((geometry().left()+geometry().right())/2,geometry().top())).manhattanLength();
            if(dis2TC<mdis) { mdis=dis2TC; mcase=4; }

            Q_ASSERT(mcase!=-1);
            _geoMode=mcase;
        }
        switch(_geoMode) {
        case 0: //TL
            geo.setX(parentWidget()->mapFromGlobal(ev->globalPos()).x()/20*20);
            geo.setY(parentWidget()->mapFromGlobal(ev->globalPos()).y()/20*20);
            break;
        case 1: //TR
            geo.setRight(parentWidget()->mapFromGlobal(ev->globalPos()).x()/20*20);
            geo.setY(parentWidget()->mapFromGlobal(ev->globalPos()).y()/20*20);
            break;
        case 2: //BL
            geo.setX(parentWidget()->mapFromGlobal(ev->globalPos()).x()/20*20);
            geo.setBottom(parentWidget()->mapFromGlobal(ev->globalPos()).y()/20*20);
            break;
        case 3: //BR
            geo.setRight(parentWidget()->mapFromGlobal(ev->globalPos()).x()/20*20);
            geo.setBottom(parentWidget()->mapFromGlobal(ev->globalPos()).y()/20*20);
            break;
        case 4: //TC
            QPoint origCenter=QPoint((geometry().left()+geometry().right())/2,geometry().top());
        {
            int moveX=(origCenter.x()-p1.x())/20*20;
            int origWidth=geometry().width();
            int moveY=(origCenter.y()-p1.y())/20*20;
            int origHeight=geometry().height();
            geo.setX((geo.x()-moveX)/20*20);
            geo.setY((geo.y()-moveY)/20*20);
            geo.setWidth(origWidth/20*20);
            geo.setHeight(origHeight/20*20);
        }
            break;
        }

        setGeometry(geo);
    }
}
