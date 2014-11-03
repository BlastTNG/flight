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

#ifndef PMAP_H
#define PMAP_H

#include <QObject>
#include "PObject.h"
#include "PStyle.h"
#include <QMap>
#include <QColor>

class PMap : public QObject, public PObject
{
    Q_OBJECT
public:
    QMap<int,QString> _map;
    friend QDataStream& operator<<(QDataStream& a,PMap& b);
    friend QDataStream& operator>>(QDataStream&a,PMap&b);
    QString get(int num) {
        QString x=_map.value(num,QString::number(num));
        while(x.contains("[")) {    //regex would kinda be cool.
            int n=x.indexOf("]",x.indexOf("["));
            if(n==-1) n=9999999;
            n-=x.indexOf("[");
            x.remove(x.indexOf("["),n+1);
        }
        return x;
    }
    PStyle* style(int num,PStyle* def) {
        QString x=_map.value(num,QString((char)31));
        if(x.size()==1&&x[0]==(char)32) {
            return def;
        }
        if(!x.contains("[style")) {
            return def;
        } else {
            QString v=x;
            v.remove(0,x.indexOf("[style=")+4);
            if((v.indexOf("]")!=-1)) {
                v.remove(v.indexOf("]"),999999);
            }
            for(int i=0;i<PStyle::_u.size();i++) {
                if(v.contains(PStyle::_u[i]->idText())) {
                    return PStyle::_u[i];
                }
            }
        }
        return def;
    }

public slots:
    void set(int num,QString val) { _map.insert(num,(val=="Hopefully, nobody will put this in as a value.")?":P":val); }
    void reset() { _map.clear(); }
};

QDataStream& operator<<(QDataStream& a,PMap& b);
QDataStream& operator>>(QDataStream&a,PMap&b);

#endif // PMAP_H
