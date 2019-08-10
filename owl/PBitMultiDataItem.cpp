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

#include "PBitMultiDataItem.h"
#include "PStyle.h"

#include <unistd.h>

void PBitMultiDataItem::gdUpdate(GetData::Dirfile* dirFile,int lastNFrames)
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

    if(_nBits < 1) {
      if(_data->text() != QString::number(indata)) {
        _data->setText(QString::number(indata));
        _serverDirty=-1;
      } else {
        --_serverDirty;
      }
    } else {
      char display[32];
      int num_bits = _nBits;
      int high_len = _highWord.length();
      int low_len = _lowWord.length();
      unsigned idata = (unsigned)indata;
      if (num_bits > 32)
        num_bits = 32;

      for (int z = 0; z < num_bits; z++)
        if (idata & (1 << ((num_bits - 1) - z)))
          display[z] = (z >= high_len) ? '1' : (char)_highWord.at(z).toLatin1();
        else
          display[z] = (z >= low_len) ? '0' : (char)_lowWord.at(z).toLatin1();
      display[num_bits]='\0';

      if (strlen(display)==1) {
        display[1] = ' ';
        display[2] = '\0';
      }
      if(_data->text() != display) {
        _data->setText(display);
        _serverDirty=-1;
      } else {
        --_serverDirty;
      }
    }
  }
}

QString PBitMultiDataItem::highWord() const { return _highWord; }
QString PBitMultiDataItem::lowWord() const { return _lowWord; }
int PBitMultiDataItem::nBits() const { return _nBits; }

void PBitMultiDataItem::setHighWord(QString highWord,bool force) {
    if(!isCurrentObject()&&!force) return;
    if(_highWord==highWord) {
        return;
    }
    _highWord=highWord;
    emit highWordChanged(highWord);
}

void PBitMultiDataItem::setLowWord(QString lowWord,bool force) {
    if(!isCurrentObject()&&!force) return;
    if(_lowWord==lowWord) {
        return;
    }
    _lowWord=lowWord;
    emit lowWordChanged(lowWord);
}

void PBitMultiDataItem::setNBits(int nBits,bool force) {
    if(!isCurrentObject()&&!force) return;
    if(_nBits==nBits) {
        return;
    }
    _nBits=nBits;
    emit nBitsChanged(nBits);
}
