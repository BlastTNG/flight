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

#ifndef PMULTIDATAITEM_H
#define PMULTIDATAITEM_H

#include "PMap.h"
#include "PAbstractDataItem.h"
#include <getdata/dirfile.h>

class PMultiDataItem : public PAbstractDataItem
{
    Q_OBJECT
    PMap* _map;
    PStyle* _lastNStyle;
public:
    friend QDataStream& operator<<(QDataStream&a,PAbstractDataItem&b);
    friend QDataStream& operator>>(QDataStream&a,PMultiDataItem &b);
    friend QVariant save(PAbstractDataItem&);
    friend void load(QVariant v,PMultiDataItem&);
    friend class PDotPal;
    friend class PMainWindow;
    PMultiDataItem(PBox*parent,QString caption) : PAbstractDataItem(parent,caption), _map(new PMap),_lastNStyle(0) {}
    PMultiDataItem(PBox*parent,PMultiDataItem* other) : PAbstractDataItem(parent,other), _map(other->_map),_lastNStyle(0) {}
    virtual void gdUpdate(GetData::Dirfile* dirFile,int lastNFrames);
    virtual PStyle* getPrevDataStyle() { return _lastNStyle?_lastNStyle:_defaultDataStyle; }
};

#endif // PMULTIDATAITEM_H
