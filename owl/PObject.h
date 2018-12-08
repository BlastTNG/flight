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

#ifndef POBJECT_H
#define POBJECT_H

#include <QStringList>
#include <QDebug>

class PMainWindow;

class PObject
{
    static bool isLoading;
public:
    int _id;
    friend QDataStream& operator<<(QDataStream& a,PObject& b);
    friend QDataStream& operator>>(QDataStream& a,PObject& b);
    friend QVariant save(PObject&);
    friend void load(QVariant v,PObject& b);
    friend void load(QVariant v,PMainWindow& b);
    friend class PMainWindow;
    static QMap<int, PObject*> _u;

    PObject(int id=qrand()) {
      if (isLoading) return;

      _id = id;
      while (_u.contains(_id)) {
        qDebug()<<"Broken random number generator?";
        id=qrand();
      }
      _u.insert(id,this);
    }

    virtual ~PObject() {
      if((_id !=0) && _u.contains(_id)) {
        _u[_id]=0;
      }
    }

    bool isCurrentObject();
    QString idText() const { return "(P"+QString::number(_id)+")"; }
    const int& id() const { return _id; }
    virtual void activate() {}
};

QDataStream& operator<<(QDataStream& a,PObject& b);
QDataStream& operator>>(QDataStream& a,PObject& b);

#endif // POBJECT_H
