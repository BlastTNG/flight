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

#include "PObject.h"
#include "PStyle.h"
#include <QWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <getdata/dirfile.h>

#ifndef PABSTRACTDATAITEM_H
#define PABSTRACTDATAITEM_H

class PMdiArea;
class PBox;

class PAbstractDataItem : public QWidget, public PObject
{
    Q_OBJECT
protected:
    QHBoxLayout* _layout;
    QLabel* _caption;
    PStyle* _captionStyle;
    PStyle* _defaultDataStyle;
    PStyle* _lastCapStyle;

    QLabel* _data;
    QString _source;
    QPoint _dragStartPos;
    int _serverDirty;
    bool _neverGood;
    bool _sourceBad;

public:
    friend class PBox;
    friend class PMainWindow;
    friend QDataStream& operator<<(QDataStream&a,PAbstractDataItem&b);
    friend QDataStream& operator>>(QDataStream&a,PAbstractDataItem&b);
    friend QVariant save(PAbstractDataItem&);
    friend void load(QVariant v,PAbstractDataItem&);

    PAbstractDataItem(PBox* parent, QString caption);
    PAbstractDataItem(PBox* parent, PAbstractDataItem* other);
    virtual void gdUpdate(GetData::Dirfile*,int){}
    virtual double gdReadRawData(GetData::Dirfile* df,int nf, bool &ok);
    QString caption() const;
    const PStyle* captionStyle() const { return _captionStyle; }
    const PStyle* defaultDataStyle() const { return _defaultDataStyle; }
    QString data() const;
    QString source() const;
    void mousePressEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent*);
    void mouseDoubleClickEvent(QMouseEvent*);
    virtual QString format() { return "???"; }
    virtual PStyle* getPrevDataStyle() { return _defaultDataStyle; }
    void resetSource() {_neverGood = true; _sourceBad = false;}
    int delaysThisCycle();
    void incrementDelays();
    static void newCycle();

public slots:
    void setCaption(QString x, bool force=0);
    void pstyleLogic();
    void setSource(QString x, bool force=0);
    virtual void setFormat(QString,bool force=0){Q_UNUSED(force)}
    virtual void activate();

signals:
    void activated();
    void textChanged(QString);
    void sourceChanged(QString);
    void formatChanged(QString);
    void styleChanged();    
};

QDataStream& operator<<(QDataStream&a,PAbstractDataItem&b);
QDataStream& operator>>(QDataStream&a,PAbstractDataItem&b);

#endif // PABSTRACTDATAITEM_H
