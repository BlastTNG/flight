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

#include <QLabel>

#include "PAbstractDataItem.h"
#include "PExtrema.h"

#ifndef PNUMBERDATAITEM_H
#define PNUMBERDATAITEM_H

class PNumberDataItem : public PAbstractDataItem
{
    Q_OBJECT
    QString _format;
    PExtrema* _extrema;
    PStyle* prevStyle;
public:
    friend QDataStream& operator<<(QDataStream&a,PAbstractDataItem&b);
    friend QDataStream& operator>>(QDataStream&a,PNumberDataItem &b);
    friend QVariant save(PAbstractDataItem&);
    friend void load(QVariant v,PNumberDataItem&);
    friend class PMainWindow;
    friend class PDotPal;
    PNumberDataItem(PBox* parent, QString caption) : PAbstractDataItem(parent,caption), _extrema(0),prevStyle(0) {}
    PNumberDataItem(PBox* parent, PNumberDataItem*b) : PAbstractDataItem(parent,b),
        _format(b->_format), _extrema(b->_extrema), prevStyle(0) {}
    void gdUpdate(GetData::Dirfile* dirFile,int lastNFrames);
    QString format() const;
    virtual PStyle* getPrevDataStyle() { return prevStyle?prevStyle:_defaultDataStyle; }

public slots:
    void setFormat(QString format,bool force=0);
};

#endif // PNUMBERDATAITEM_H
