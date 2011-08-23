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

#ifndef PTIMEDATAITEM_H
#define PTIMEDATAITEM_H

#include "PMap.h"
#include "PAbstractDataItem.h"
#include <getdata/dirfile.h>

class PTimeDataItem : public PAbstractDataItem
{
    Q_OBJECT
    QString _format;
    PStyle* _pstyle;
public:
    friend QDataStream& operator<<(QDataStream&a,PAbstractDataItem&b);
    friend QDataStream& operator>>(QDataStream&a,PTimeDataItem &b);
    friend class PMainWindow;
    PTimeDataItem(PBox*p,QString caption) : PAbstractDataItem(p,caption) {}
    void gdUpdate(GetData::Dirfile* dirFile,int lastNFrames);
    QString format() const;

public slots:
    void setFormat(QString format,bool force=0);
};

#endif // PTIMEDATAITEM_H
