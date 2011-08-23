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

#include "PTimeDataItem.h"
#include <ctime>

void PTimeDataItem::gdUpdate(GetData::Dirfile* dirFile,int lastNFrames)
{
    double indata[20];
    // Read in from disk
    int i=0;
    while (dirFile->GetData(_source.toStdString().c_str(),
                         lastNFrames-1, 0, 0, 1, // 1 sample from frame nf-1
                         GetData::Float64, (void*)(indata))==0) {
        if(++i==5) {
            if(_data->text()!="bad src") {
                _data->setText("bad src");
                _serverDirty=-1;
            } else --_serverDirty;
            return;
        } else {
//            usleep(10000);
        }
    } {
        if(_pstyle!=_defaultDataStyle||_defaultDataStyle->_dirty) {
            applyStyle(_data,_defaultDataStyle);
            _pstyle=_defaultDataStyle;
        }
        char tmp[255];
        time_t timetmp = (time_t)*indata;
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
