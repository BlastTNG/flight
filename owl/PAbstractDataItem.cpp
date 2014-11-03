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

#include "PAbstractDataItem.h"
#include "PMainWindow.h"
#include <QMouseEvent>
#include <QApplication>
#include <QDebug>
#include <QPlastiqueStyle>

PAbstractDataItem::PAbstractDataItem(PBox* parent, QString caption) : QWidget(parent), _layout(new QHBoxLayout()),
    _caption(new QLabel(caption)), _captionStyle(PStyle::noStyle), _defaultDataStyle(PStyle::noStyle),
    _lastCapStyle(0), _data(new QLabel(tr("Loading"))), _serverDirty(-1), _neverGood(true), _sourceBad(false)
{
    connect(PStyleNotifier::me,SIGNAL(change()),this,SLOT(pstyleLogic()));
    setLayout(_layout);
    _layout->setMargin(0);
    _layout->setSpacing(0);
    _layout->addWidget(_caption);
    _layout->addWidget(_data);
    parent->_dirty=1;
    QPlastiqueStyle* ps=new QPlastiqueStyle;
    _caption->setStyle(ps);
    _data->setStyle(ps);
}

PAbstractDataItem::PAbstractDataItem(PBox* parent, PAbstractDataItem* other) : QWidget(parent), _layout(new QHBoxLayout()),
    _caption(new QLabel(other->caption())), _captionStyle(other->_captionStyle), _defaultDataStyle(other->_defaultDataStyle),
    _lastCapStyle(0), _data(new QLabel(tr("Loading"))), _source(other->_source), _serverDirty(-1)
{
    connect(PStyleNotifier::me,SIGNAL(change()),this,SLOT(pstyleLogic()));
    setLayout(_layout);
    _layout->setMargin(0);
    _layout->setSpacing(0);
    _layout->addWidget(_caption);
    _layout->addWidget(_data);
    parent->_dirty=1;
    QPlastiqueStyle* ps=new QPlastiqueStyle;
    _caption->setStyle(ps);
    _data->setStyle(ps);
}

QString PAbstractDataItem::caption() const
{
    return _caption->text();
}

QString PAbstractDataItem::data() const
{
    return _data->text();
}

QString PAbstractDataItem::source() const
{
    return _source;
}

void PAbstractDataItem::mousePressEvent(QMouseEvent *event)
{
    if (PMainWindow::me->mouseInactive()) return;

    emit activated();

    if(event) {
        if (event->button() == Qt::LeftButton) {
            _dragStartPos = event->pos();
        }
    }
}

void PAbstractDataItem::mouseMoveEvent(QMouseEvent *event)
{
    if (PMainWindow::me->mouseInactive()) return;

    if (!(event->buttons() & Qt::LeftButton))
        return;
    if ((event->pos() - _dragStartPos).manhattanLength() < QApplication::startDragDistance())
        return;

    QDrag *drag = new QDrag(this);
    QMimeData *mimeData = new QMimeData;

    mimeData->setData("text/plain", (PMainWindow::me->_dirfileFilename+"#"+_source).toAscii());
    mimeData->setData("application/x-owlid", QByteArray::number(id()));
    drag->setMimeData(mimeData);

    drag->exec(Qt::CopyAction | Qt::MoveAction);
}

void PAbstractDataItem::setCaption(QString x, bool force) {
    if(!isCurrentObject()&&!force) {
        return;
    }

    if(_caption->text()==x) {
        return;
    }
    _caption->setText(x);
    emit textChanged(x);
    dynamic_cast<PBox*>(parent())->_dirty=1;
}

void PAbstractDataItem::pstyleLogic() {
    if(_captionStyle!=_lastCapStyle|| //the reason this works is because PStyleChooser works via reference to ptr
            _captionStyle->_dirty) {
        applyStyle(_caption,_captionStyle);
        _lastCapStyle=_captionStyle;
    }
}

void PAbstractDataItem::setSource(QString x,bool force) {
    if(!isCurrentObject()&&!force) return;
    if(x==_source) {
        return;
    }
    _source=x;
    _sourceBad = false;
    _neverGood = true;
    emit sourceChanged(x);
}

void PAbstractDataItem::activate()
{
    emit activated();
}
