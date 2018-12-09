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

#include "PNumberDataItem.h"
#include <stdio.h>
#include <QDebug>
#include <unistd.h>


void PNumberDataItem::gdUpdate(GetData::Dirfile* dirFile,int lastNFrames)
{
    double indata;
    bool ok;

    indata = gdReadRawData(dirFile, lastNFrames, ok);

    if (ok) {
        PStyle* style=_extrema?_extrema->formatForValue(indata,_defaultDataStyle):_defaultDataStyle;

        if(prevStyle!=style|| //the reason this works is because PStyleChooser works via reference to ptr
                style->_dirty) {
            applyStyle(this,style);
            prevStyle=style;
        }

        if(_format.isEmpty()) {
            if(_data->text() != QString::number(indata)) {
                _data->setText(QString::number(indata));
                _serverDirty=-1;
            } else {
                --_serverDirty;
            }
        } else {
            const char* asciiFormat=_format.toStdString().c_str();

            bool is_int = 0;
            bool is_binary = 0;
            for (int i=0; asciiFormat[i]!='\0'; i++) {
                char c = asciiFormat[i];
                switch (c) {
                case 'x':
                case 'X':
                case 'o':
                case 'd':
                    is_int = 1;
                    break;
                case 'b':
                    is_binary = 1;
                    break;
                default:
                    break;
                }
            }
            char display[512];
            if (is_int) {
                sprintf(display, asciiFormat, (int)indata);
            } else if (is_binary) {
                int idata = (int)indata;
                long int num_bits = strtol((asciiFormat)+1, NULL, 0);
                if (num_bits < 1 || num_bits > 32) {
                    num_bits = 8;
                }
                for (int z = 0; z < num_bits; z++) {
                    display[z] = (idata & (1 << ((num_bits - 1) - z))) ? '1' : '0';
                }
                display[num_bits]='\0';
            } else {
                sprintf(display, asciiFormat, indata);
            }
            if (strlen(display)==1) {
                display[1] = ' ';
                display[2] = '\0';
            }
            if(_data->text()!=display) {
                _data->setText(display);
                _serverDirty=-1;
            } else {
              --_serverDirty;
            }
        }
    }
}

QString PNumberDataItem::format() const {
    return _format;
}

void PNumberDataItem::setFormat(QString format,bool force) {
    if(!isCurrentObject()&&!force) return;
    if(_format==format) {
        return;
    }
    _format=format;
    emit formatChanged(format);
}
