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

#include "PTimeDataItem.h"
#include <ctime>
#include <unistd.h>


void PTimeDataItem::gdUpdate(GetData::Dirfile* dirFile,int lastNFrames)
{
    double indata;
    bool ok;

    indata = gdReadRawData(dirFile, lastNFrames, ok);

    //sanity check:
    if ((indata < 0.0) || (indata > INT_MAX)) {
      // this is not a time.  Don't use it!
      indata = 0.0;
    }
    if (ok) {
        PStyle* style = _extrema
          ? _extrema->formatForValue(time(NULL) - indata,_defaultDataStyle)
          : _defaultDataStyle;

        if(prevStyle!=style|| //the reason this works is because PStyleChooser works via reference to ptr
                style->_dirty) {
            applyStyle(this,style);
            prevStyle=style;
        }
        char tmp[255];
        time_t timetmp = (time_t)indata;
        struct tm* currTime;

        currTime = gmtime(&timetmp);
        strftime(tmp, 255, _format.toStdString().c_str(), currTime);
        if(_data->text()!=tr(tmp)) {
            _data->setText(tr(tmp));
            _serverDirty=-1;
        } else --_serverDirty;
    }
}

QString PTimeDataItem::format() const {
    return _format;
}

void PTimeDataItem::setFormat(QString format,bool force) {
    if(!isCurrentObject()&&!force) return;
    if(_format==format) {
        return;
    }
    _format=format;
    emit formatChanged(format);
}
