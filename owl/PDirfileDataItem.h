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
#include <QWidget>
#include <QHBoxLayout>
#include <QLabel>
#include "getdata/dirfile.h"
#include "PAbstractDataItem.h"

#ifndef PDIRFILEDATAITEM_H
#define PDIRFILEDATAITEM_H

class PDirfileDataItem : public PAbstractDataItem
{
    Q_OBJECT
public:
    friend QDataStream& operator<<(QDataStream&a,PAbstractDataItem&b);
    friend class PMainWindow;
    PDirfileDataItem(PBox*p,QString caption) : PAbstractDataItem(p,caption) {}
    PDirfileDataItem(PBox*p,PDirfileDataItem* other) : PAbstractDataItem(p,other) {}
    virtual void gdUpdate(GetData::Dirfile* dirFile,int lastNFrames);
};

#endif // PDIRFILEDATAITEM_H
