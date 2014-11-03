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

#include "PDirfileDataItem.h"
#include "PMainWindow.h"
#include <QMouseEvent>
#include <QApplication>
#include <QDebug>

void PDirfileDataItem::gdUpdate(GetData::Dirfile *dirFile, int)
{
    QString x=dirFile->Name();
    if(x.endsWith('/')) x.chop(1);
    if(x.contains('/')) {
        x.remove(0,x.lastIndexOf('/')+1);
    }
    if(_data->text()!=x) {
        _data->setText(x);
        _serverDirty=-1;
    } else --_serverDirty;
}
