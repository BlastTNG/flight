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

#ifndef PBITMULTIDATAITEM_H
#define PBITMULTIDATAITEM_H

#include "PExtremaDataItem.h"

class PBitMultiDataItem : public PExtremaDataItem
{
    Q_OBJECT
    int _nBits;
    QString _highWord;
    QString _lowWord;
public:
    friend QDataStream& operator<<(QDataStream&a,PAbstractDataItem&b);
    friend QDataStream& operator>>(QDataStream&a,PBitMultiDataItem &b);
    friend QVariant save(PAbstractDataItem&);
    friend void load(QVariant v,PBitMultiDataItem&);
    friend class PDotPal;
    friend class PMainWindow;
    PBitMultiDataItem(PBox*parent,QString caption) : PExtremaDataItem(parent,caption),
        _nBits(0) {}
    PBitMultiDataItem(PBox*parent,PBitMultiDataItem* other) : PExtremaDataItem(parent,other),
        _nBits(other->_nBits), _highWord(other->_highWord), _lowWord(other->_lowWord) {}
    virtual void gdUpdate(GetData::Dirfile* dirFile,int lastNFrames);
    QString highWord() const;
    QString lowWord() const;
    int nBits() const;

public slots:
    void setHighWord(QString highWord,bool force=0);
    void setLowWord(QString lowWord,bool force=0);
    void setNBits(int nBits,bool force=0);

signals:
    void highWordChanged(QString);
    void lowWordChanged(QString);
    void nBitsChanged(int);
};

#endif // PBITMULTIDATAITEM_H
