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

#ifndef POBJECT_H
#define POBJECT_H

#include <QStringList>
#include <QDebug>

class PObject
{
    static int _lastId;
    int _id;
public:
    friend QDataStream& operator<<(QDataStream& a,PObject& b);
    friend QDataStream& operator>>(QDataStream& a,PObject& b);
    friend class PMainWindow;
    static QList<PObject*> _u;

    PObject(int id=++_lastId) : _id(id) { _lastId=qMax(_lastId,id); _u.push_back(this); }
    virtual ~PObject() { if(_u.contains(this)) _u[_u.indexOf(this)]=0; else qDebug()<<"Removing already removed item!"; }

    bool isCurrentObject();
    QString idText() const { return "(P"+QString::number(_id)+")"; }
    const int& id() const { return _id; }
    virtual void activate() {}
};

QDataStream& operator<<(QDataStream& a,PObject& b);
QDataStream& operator>>(QDataStream& a,PObject& b);

#endif // POBJECT_H
