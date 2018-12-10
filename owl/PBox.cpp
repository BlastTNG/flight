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

#include "PBox.h"
#include "PMainWindow.h"
#include "PTimeDataItem.h"
#include "PDirfileDataItem.h"
#include "PBoxTitle.h"
#include <QInputDialog>
#include <QDesktopServices>
#include <QMimeData>

PBox::PBox(QString boxTitle, const QFont &F, QWidget*p) : QFrame(p), _pstyle(PStyle::noStyle), _lastPStyle(0),_layout(new QVBoxLayout()),
    _boxTitle(boxTitle), _geoMode(-1), _dirty(1)
{
    bool ok=0;
    for(int i=0;i<PStyle::_u.size();i++) {
        Q_ASSERT(PStyle::_u[i]);
        if(PStyle::_u[i]->name()=="defbox") {
            ok=1;
            _pstyle=PStyle::_u[i];
            break;
        }
    }

    setFont(F);
    _H = fontMetrics().height();

    _pstyle=ok?_pstyle:new PStyle("defbox",1,0,"white","blue",1);

    setFrameStyle(QFrame::NoFrame | QFrame::Plain);

    setLineWidth(1);
    int margin = fontMetrics().height()/3;
    setContentsMargins(0,0,0,0);
    setMidLineWidth(1);
    //setFrameShadow(QFrame::Plain);
    _pbt=new PBoxTitle;
    _pbt->setText("[change me]");
    _pbt->setFont(F);
    addTitle(_pbt);
    connect(PStyleNotifier::me,SIGNAL(change()),this,SLOT(styleLogic()));
    setWindowTitle(boxTitle);
    setLayout(_layout);
    _layout->setSpacing(0);
    setMinimumSize(_H*2,_H*3);
    setAcceptDrops(1);
    _layout->setContentsMargins(margin,0,margin,0);
    _layout->addSpacerItem(new QSpacerItem(0,0,QSizePolicy::Minimum,QSizePolicy::MinimumExpanding));

    // QRect geo=geometry();
    // geo.setX(((int)geo.x()/_H)*_H);
    // geo.setY(((int)geo.y()/_H)*_H);
    // geo.setWidth(((int)geo.width()/_H)*_H+0);
    // geo.setHeight(((int)geo.height()/_H)*_H+0);
    // if(geo!=geometry()) {
    //     setGeometry(geo);
    // }
    setAutoFillBackground(1);
    setMouseTracking(1);
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

void PBox::setBoxTitle(const QString& boxTitle, bool force)
{
  if (boxTitle.isEmpty()) {
    _pbt->setFixedHeight(0);
  }

    if (!isCurrentObject() && !force) return;

    if(_boxTitle==boxTitle) return;

    _boxTitle=boxTitle;
    _pbt->setText(boxTitle);

    emit textChanged(boxTitle);
}

void PBox::gdUpdate(GetData::Dirfile* dirFile,int lastNFrames)
{
    for(int i=0;i<_dataItems.size();i++) {
        _dataItems[i]->gdUpdate(dirFile,lastNFrames);
    }
    if(_dirty) {
        int maxCapWidth = 0;
        int maxCapHeight = 0;
        for(int i=0;i<_dataItems.size();i++) {
            _dataItems[i]->gdUpdate(dirFile, lastNFrames);
            QSize size = _dataItems[i]->_caption->fontMetrics().size(0, _dataItems[i]->_caption->text());

            maxCapWidth=qMax(maxCapWidth, size.width());
            maxCapHeight = qMax(maxCapHeight, _dataItems[i]->_caption->fontMetrics().height());
        }
        for(int i=0;i<_dataItems.size();i++) {
            if(_dataItems[i]->_caption->width()!=maxCapWidth) {
                _dataItems[i]->_caption->setFixedWidth(maxCapWidth);
            }
            _dataItems[i]->_caption->setFixedHeight(maxCapHeight);
            _dataItems[i]->_data->setFixedHeight(maxCapHeight);
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
        if(row&&row<6&&!col) {    //HACKHACKHACKHACK!!!
            ev->acceptProposedAction();
        }
    }
    else if(ev->mimeData()->hasFormat("application/x-owlid")) {
        ev->acceptProposedAction();
    }
}

int PBox::getWhereToInsert(QPoint p) {
    int pos=-1;
    for(int i=0;i<_dataItems.size();i++) {
        if(!i&&_dataItems[i]->y()>p.y()) {
            pos=0;
            break;
        }
        if(_dataItems[i]->geometry().contains(p)) {
            if(p.y()>_dataItems[i]->geometry().center().y()) {
                pos=i+1;
            } else {
                pos=i;
            }
            break;
        }
    }
    return pos;
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
        int pos=getWhereToInsert(ev->pos());
        if(pos!=-1) ++pos;
        if(row&&row<6&&!col) {    //HACKHACKHACKHACK!!!
            switch(row) {
            case 1:
                addProperty("Number",pos);
                break;
            case 2:
                addProperty("Multi",pos);
                break;
            case 3:
                addProperty("Time/Date",pos);
                break;
            case 4:
                addProperty("Dirfile",pos);
                break;
            case 5:
                addProperty("BitMulti",pos);
                break;
            default:
                addProperty("Item",pos);
                break;
            }
        }
    }
    else if(ev->mimeData()->hasFormat("application/x-owlid")) {
        int oid=ev->mimeData()->data("application/x-owlid").toInt();
        for(int i=0;i<PObject::_u.size();i++) {
            if(!PObject::_u.values()[i]) continue;
            if(PObject::_u.values()[i]->id()==oid&&dynamic_cast<PAbstractDataItem*>(PObject::_u.values()[i])) {
                QString answer;
                if(ev->keyboardModifiers()&Qt::ControlModifier) {
                    answer="copy";
                } else if(ev->keyboardModifiers()&Qt::ShiftModifier) {
                    answer="move";
                } else {
                    QStringList a;
                    a.push_back("move (shift+drag)");
                    a.push_back("copy (ctrl+drag)");
                    bool ok;
                    answer=QInputDialog::getItem(this,"Move or Copy?","Choose an action:",a,0,false,&ok);
                    if(!ok) return;
                }
                int pos=getWhereToInsert(ev->pos());
                if(pos!=-1) ++pos;
                if(answer.startsWith("copy")) {
                    if(dynamic_cast<PNumberDataItem*>(PObject::_u.values()[i])) {
                        addProperty(new PNumberDataItem(this,dynamic_cast<PNumberDataItem*>(PObject::_u.values()[i])),pos);
                    } else if(dynamic_cast<PMultiDataItem*>(PObject::_u.values()[i])) {
                        addProperty(new PMultiDataItem(this,dynamic_cast<PMultiDataItem*>(PObject::_u.values()[i])),pos);
                    } else if(dynamic_cast<PBitMultiDataItem*>(PObject::_u.values()[i])) {
                        addProperty(new PBitMultiDataItem(this,dynamic_cast<PBitMultiDataItem*>(PObject::_u.values()[i])),pos);
                    } else if(dynamic_cast<PTimeDataItem*>(PObject::_u.values()[i])) {
                        addProperty(new PTimeDataItem(this,dynamic_cast<PTimeDataItem*>(PObject::_u.values()[i])),pos);
                    } else if(dynamic_cast<PDirfileDataItem*>(PObject::_u.values()[i])) {
                        addProperty(new PDirfileDataItem(this,dynamic_cast<PDirfileDataItem*>(PObject::_u.values()[i])),pos);
                    } else {
                        qDebug()<<"Cannot copy data item of unknown type!";
                    }
                } else if(answer.startsWith("move")) {
                    dynamic_cast<PAbstractDataItem*>(PObject::_u.values()[i])->_caption->setFixedWidth(1);
                    PBox*x=dynamic_cast<PBox*>(dynamic_cast<PAbstractDataItem*>(PObject::_u.values()[i])->parentWidget());   //assert?
                    x->_layout->removeWidget(dynamic_cast<PAbstractDataItem*>(PObject::_u.values()[i]));
                    x->_dataItems.removeOne(dynamic_cast<PAbstractDataItem*>(PObject::_u.values()[i]));
                    addProperty(dynamic_cast<PAbstractDataItem*>(PObject::_u.values()[i]),pos);
                    _dirty=1;
                }
                return;
            }
        }
    }
}

void PBox::addProperty(QString property,int pos)
{
    if(property=="Number") {
        addProperty(new PNumberDataItem(this, property),pos);
    } else if(property=="BitMulti") {
        addProperty(new PBitMultiDataItem(this, property),pos);
    } else if(property=="Multi") {
        addProperty(new PMultiDataItem(this, property),pos);
    } else if(property=="Time/Date") {
        addProperty(new PTimeDataItem(this, property),pos);
    } else if(property=="Dirfile") {
        addProperty(new PDirfileDataItem(this, property),pos);
    } else {
        qWarning()<<"Adding pure abstract data item";
        addProperty(new PAbstractDataItem(this, property),pos);
    }
}

void PBox::addProperty(PAbstractDataItem* padi,int pos)
{
    if (pos==-1) {
      _dataItems.push_back(padi);
    } else { /* if != -1, then pos has biased by one by the caller */
      _dataItems.insert(pos - 1,padi);
    }

    _layout->insertWidget((pos==-1)?_dataItems.size():pos, padi); //not -1 because of title
    padi->setFixedHeight(padi->fontMetrics().height());
    padi->show();
    emit newChild(padi);
}

void PBox::addTitle(PBoxTitle* pbt)
{
    if (pbt->text().isEmpty()) {
      printf("Empty Title!\n");
    }

    _layout->insertWidget(0,pbt);
    pbt->setFixedHeight(pbt->fontMetrics().height());
    pbt->show();
}

void PBox::mousePressEvent(QMouseEvent *e)
{
    if (PMainWindow::me->mouseInactive()) return;

    _geoMode=-1;
    emit activated();
    QWidget::mousePressEvent(e);
}

void PBox::mouseReleaseEvent(QMouseEvent *)
{
    if (PMainWindow::me->mouseInactive()) return;

    _geoMode=-1;
}

void PBox::mouseMoveEvent(QMouseEvent *ev)
{
    if (PMainWindow::me->mouseInactive()) {
        setCursor(QCursor(Qt::ArrowCursor));
        return;
    }

    QPoint p1=parentWidget()->mapFromGlobal(ev->globalPos());   //setGeometry refers to parent's (x,y)
    QRect geo=geometry();
    bool resize=(ev->buttons()&Qt::LeftButton);
    if(!resize) {
        _geoMode=-1;
    }
    if(_geoMode==-1) {
        int mdis=99999999;
        int mcase=-1;

        int dis2TL=(p1-geometry().topLeft()).manhattanLength();
        if(dis2TL<mdis&&(resize||dis2TL<_H)) { mdis=dis2TL; mcase=0; }

        int dis2TR=(p1-geometry().topRight()).manhattanLength();
        if(dis2TR<mdis&&(resize||dis2TR<_H)) { mdis=dis2TR; mcase=1; }

        int dis2BL=(p1-geometry().bottomLeft()).manhattanLength();
        if(dis2BL<mdis&&(resize||dis2BL<_H)) { mdis=dis2BL; mcase=2; }

        int dis2BR=(p1-geometry().bottomRight()).manhattanLength();
        if(dis2BR<mdis&&(resize||dis2BR<_H)) { mdis=dis2BR; mcase=3; }

        int dis2TC=(p1-QPoint((geometry().left()+geometry().right())/2,geometry().top())).manhattanLength();
        if(dis2TC<mdis&&(resize||dis2TC<_H)) { mdis=dis2TC; mcase=4; }

        Q_ASSERT(!resize||(mcase!=-1));
        _geoMode=mcase;
    }
    switch(_geoMode) {
    case 0: //TL
        setCursor(QCursor(Qt::SizeFDiagCursor));
        geo.setX(parentWidget()->mapFromGlobal(ev->globalPos()).x()/_H*_H);
        geo.setY(parentWidget()->mapFromGlobal(ev->globalPos()).y()/_H*_H);
        break;
    case 1: //TR
        setCursor(QCursor(Qt::SizeBDiagCursor));
        geo.setRight(parentWidget()->mapFromGlobal(ev->globalPos()).x()/_H*_H+0);
        geo.setY(parentWidget()->mapFromGlobal(ev->globalPos()).y()/_H*_H);
        break;
    case 2: //BL
        setCursor(QCursor(Qt::SizeBDiagCursor));
        geo.setX(parentWidget()->mapFromGlobal(ev->globalPos()).x()/_H*_H);
        geo.setBottom(parentWidget()->mapFromGlobal(ev->globalPos()).y()/_H*_H+0);
        break;
    case 3: //BR
        setCursor(QCursor(Qt::SizeFDiagCursor));
        geo.setRight(parentWidget()->mapFromGlobal(ev->globalPos()).x()/_H*_H+0);
        geo.setBottom(parentWidget()->mapFromGlobal(ev->globalPos()).y()/_H*_H+0);
        break;
    case 4: //TC
        if(ev->buttons()&Qt::LeftButton) {
            setCursor(QCursor(Qt::ClosedHandCursor));
        } else {
            setCursor(QCursor(Qt::OpenHandCursor));
        }
    {
        QPoint origCenter=QPoint((geometry().left()+geometry().right())/2,geometry().top());
        int moveX=(origCenter.x()-p1.x())/_H*_H;
        int origWidth=geometry().width();
        int moveY=(origCenter.y()-p1.y())/_H*_H;
        int origHeight=geometry().height();
        geo.setX((geo.x()-moveX)/_H*_H);
        geo.setY((geo.y()-moveY)/_H*_H);
        geo.setWidth(origWidth/_H*_H);
        geo.setHeight(origHeight/_H*_H);
    }
        break;

    default:
        setCursor(QCursor(Qt::ArrowCursor));

    }

    if(ev->buttons()&Qt::LeftButton) {
        setGeometry(geo);
    }
}
