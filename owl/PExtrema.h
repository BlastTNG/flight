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

#ifndef PEXTREMA_H
#define PEXTREMA_H

#include "PObject.h"
#include "PStyle.h"
#include <QObject>
#include <QColor>

class PExtrema : public QObject, public PObject
{
    Q_OBJECT
    QString _name;
public:
    friend QDataStream& operator<<(QDataStream&,PExtrema&b);
    friend QDataStream& operator>>(QDataStream&,PExtrema&b);
    friend QVariant save(PExtrema&);
    friend void load(QVariant v,PExtrema&);
    static QList<PExtrema*> _u;
    float _xhigh,_high,_low,_xlow;
    PStyle* _sxhigh,*_shigh,*_slow,*_sxlow;
    const QString& name() const { return _name; }

    PExtrema() : _xhigh(2.0f),_high(1.0f),_low(-1.0f),_xlow(-2.0f)
    {
        _u.push_back(this);
        bool xmaxok=0,maxok=0,minok=0,xminok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            Q_ASSERT(PStyle::_u[i]);
            if(PStyle::_u[i]->name()=="xhigh") {
                _sxhigh=PStyle::_u[i];
                xmaxok=1;
            } else if(PStyle::_u[i]->name()=="high") {
                _shigh=PStyle::_u[i];
                maxok=1;
            } else if(PStyle::_u[i]->name()=="low") {
                _slow=PStyle::_u[i];
                minok=1;
            } else if(PStyle::_u[i]->name()=="xlow") {
                _sxlow=PStyle::_u[i];
                xminok=1;
            }
        }
        _sxhigh=xmaxok?_sxhigh:new PStyle("xhigh",1,0,"#ffaaaa","#000055",1);
        _shigh=maxok?_shigh:new PStyle("high",1,0,"white","#ff0000",1);
        _slow=minok?_slow:new PStyle("low",1,0,"white","#00bb00",1);
        _sxlow=xminok?_sxlow:new PStyle("xlow",1,0,"#00ff00","#000033",1);
    }

    ~PExtrema() { _u.removeOne(this); }
    PStyle* formatForValue(float val,PStyle* defaultValue);

public slots:
    void setName(QString name);

signals:
    void nameChanged(QString name);
};

#endif // PEXTREMA_H
