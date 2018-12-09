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

#include <QWidget>
#include "PObject.h"
#include "PAbstractDataItem.h"
#include "PNumberDataItem.h"
#include "PBitMultiDataItem.h"
#include "PMultiDataItem.h"
#include "PStyle.h"
#include <QDataStream>
#include <QDragEnterEvent>
#include <QMap>
#include <getdata/dirfile.h>

#ifndef PBOX_H
#define PBOX_H

class PBoxTitle;

class PBox : public QFrame, public PObject
{
    Q_OBJECT
    PStyle* _pstyle;
    PStyle* _lastPStyle;
    QVBoxLayout* _layout;
    QString _boxTitle;
    PBoxTitle* _pbt;
    int _geoMode;
    friend class PBoxTitle;
    int _H;

public:
    bool _dirty;
    friend QDataStream& operator<<(QDataStream&a,PBox&b);
    friend QDataStream& operator>>(QDataStream&a,PBox&b);
    friend QVariant save(PBox&);
    friend void load(QVariant v,PBox&b);
    friend class PDotPal;
    friend class PMainWindow;

    QList<PAbstractDataItem*> _dataItems;
    PBox(QString boxTitle, const QFont &F, QWidget*p=0);
    ~PBox();
    const QString& boxTitle() const { return _boxTitle; }
    const PStyle* getStyle() const { return _pstyle; }

    int getWhereToInsert(QPoint p);

public slots:
    void setBoxTitle(const QString& boxTitle,bool force=0);
    void gdUpdate(GetData::Dirfile* dirFile,int lastNFrames);
    virtual void activate() { emit activated(); }
    void checkActivationState();
    void styleLogic();

signals:
    void textChanged(QString);

protected:
    void dragEnterEvent(QDragEnterEvent *ev);
    void dropEvent(QDropEvent *ev);
    void addProperty(QString property,int pos=-1);
    void addProperty(PAbstractDataItem* padi,int pos=-1);
    void addTitle(PBoxTitle* _pbt);
    void mousePressEvent(QMouseEvent *);
    void mouseReleaseEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *);

signals:
    void activated();
    void newChild(PAbstractDataItem* padi);
};

QDataStream& operator<<(QDataStream&a,PBox&b);
QDataStream& operator>>(QDataStream&a,PBox&b);

#endif // PBOX_H
