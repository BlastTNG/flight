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

#include <QWidget>
#include <QDataStream>
#include <QLabel>
#include <QHBoxLayout>
#include <getdata/dirfile.h>
#include "PObject.h"
#include <QMouseEvent>
#include <QPoint>

#ifndef OWLBOX_H
#define OWLBOX_H

class Owl : public QWidget, public PObject
{
    Q_OBJECT
    int _lastNFrames;
    QPixmap* _pixMaps[5];
    QLabel* _label;
    int _stage;
    QPoint _moveThingy;
public:
    friend QDataStream& operator<<(QDataStream&a,Owl&b);
    friend QDataStream& operator>>(QDataStream&a,Owl&b);

    Owl();
    ~Owl();
    void gdUpdate(GetData::Dirfile* dirFile,int lastNFrames);
    QSize sizeHint();
    const int& stage() const;
    void mousePressEvent(QMouseEvent *ev) { _moveThingy=mapFromGlobal(ev->globalPos()); }
    void mouseReleaseEvent(QMouseEvent *) { _moveThingy=QPoint(-1,-1); }
    void mouseMoveEvent(QMouseEvent *);
};

QDataStream& operator<<(QDataStream&a,Owl&b);
QDataStream& operator>>(QDataStream&a,Owl&b);

#endif // OWLBOX_H
