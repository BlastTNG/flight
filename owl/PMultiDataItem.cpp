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

#include "PMultiDataItem.h"
#include "PStyle.h"

#include <unistd.h>

void PMultiDataItem::gdUpdate(GetData::Dirfile* dirFile,int lastNFrames)
{
  double indata;
  bool ok;

  indata = gdReadRawData(dirFile, lastNFrames, ok);

  if (ok) {
    QString x=_map->get(indata);
    if (_data->text()!=x) {
      _data->setText(x);
      _serverDirty=-1;
    } else {
      _serverDirty--;
    }

    PStyle* nstyle=_map->style(indata,_defaultDataStyle);

    if(nstyle!=_lastNStyle|| //the reason this works is because PStyleChooser works via reference to ptr
       nstyle->_dirty) {
      applyStyle(this,nstyle);
      _lastNStyle=nstyle;
    }
  }
}
