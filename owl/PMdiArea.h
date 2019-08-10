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
#include "POwlAnimation.h"
#include "PBox.h"

#include <QMdiArea>
#include <QDragEnterEvent>
#include <QMdiSubWindow>
#include <QDebug>

#ifndef PMDIAREA_H
#define PMDIAREA_H

/* Despite the name, this has nothing to do with QMdiArea.
 * It was a QMdiArea before, but QMdiArea was not efficient
 * enough.
 */
class PMdiArea : public QWidget, public PObject
{
    Q_OBJECT
public:
    explicit PMdiArea(QWidget *parent = 0) : QWidget(parent)
    {
        setAcceptDrops(1);
        QPalette pal=palette();
        pal.setColor(backgroundRole(),"white");
        setPalette(pal);
        setAutoFillBackground(1);
        _H = fontMetrics().height();
    }
    void set_H(int h_in) {_H = h_in;}

protected:
    void dragEnterEvent(QDragEnterEvent *ev);
    void dropEvent(QDropEvent *ev);
    int _H;

public slots:
    void createPBox(int x=0,int y=0,PBox*c_pbox=0);
    void createOwl(int x=0,int y=0,POwlAnimation*c_owl=0);

signals:
    void newBox(PBox*);
    void newOwl(POwlAnimation*);
};

#endif // PMDIAREA_H
