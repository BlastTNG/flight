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

#include "POwlAnimation.h"
#include "PAbstractDataItem.h"
#include "PDirfileDataItem.h"
#include "PBox.h"
#include "PExtrema.h"
#include "PMainWindow.h"
#include "PMap.h"
#include "PMdiArea.h"
#include "PMdiArea.h"
#include "PMultiDataItem.h"
#include "PNumberDataItem.h"
#include "PTimeDataItem.h"
#include "PObject.h"
#include <QTimer>
#include <QDebug>
#include <QMessageBox>

int revId=-1;
int _lastId=0;  //in order to not break anything

QDataStream& operator<<(QDataStream& a,PObject& b)
{
    return a << (QString)"begin PObject"<<
                b._id<<_lastId;
}

QVariant save(PObject& b)
{
    QMap<QString,QVariant> ret;
    ret.insert("_id",b._id);
    return ret;
}

QDataStream& operator>>(QDataStream& a,PObject& b)
{
    QString verify;
    a>>verify;
    Q_ASSERT(verify=="begin PObject");

    return a>>b._id>>_lastId;
}

void load(QVariant s,PObject& b)
{
    b._id=s.toMap()["_id"].toInt();
    PObject::_u[b._id]=&b;
}

QDataStream& operator<<(QDataStream& a,PExtrema& b)
{
    a << (QString)"begin PExtrema"<<
                b._name<<

                b._xhigh<<b._high<<b._low<<b._xlow;

    a<<((bool)(b._sxhigh==PStyle::noStyle));
    if(b._sxhigh!=PStyle::noStyle) {
        a<<b._sxhigh->idText();
    }

    a<<((bool)(b._shigh==PStyle::noStyle));
    if(b._shigh!=PStyle::noStyle) {
        a<<b._shigh->idText();
    }

    a<<((bool)(b._slow==PStyle::noStyle));
    if(b._slow!=PStyle::noStyle) {
        a<<b._slow->idText();
    }

    a<<((bool)(b._sxlow==PStyle::noStyle));
    if(b._sxlow!=PStyle::noStyle) {
        a<<b._sxlow->idText();
    }

    a<<*(PObject*)(&b);

    return a;
}

QVariant save(PExtrema &b)
{
    QMap<QString,QVariant> ret;
    ret.insert("_name",b._name);
    ret.insert("_xhigh",b._xhigh);
    ret.insert("_high",b._high);
    ret.insert("_low",b._low);
    ret.insert("_xlow",b._xlow);

    if(b._sxhigh!=PStyle::noStyle) {
        ret.insert("_sxhigh",b._sxhigh->idText());
    } else {
        ret.insert("_sxhigh","NO STYLE");
    }

    if(b._shigh!=PStyle::noStyle) {
        ret.insert("_shigh",b._shigh->idText());
    } else {
        ret.insert("_shigh","NO STYLE");
    }

    if(b._slow!=PStyle::noStyle) {
        ret.insert("_slow",b._slow->idText());
    } else {
        ret.insert("_slow","NO STYLE");
    }

    if(b._sxlow!=PStyle::noStyle) {
        ret.insert("_sxlow",b._sxlow->idText());
    } else {
        ret.insert("_sxlow","NO STYLE");
    }

    ret.insert("PObject",save(*(PObject*)(&b)));
    return ret;
}

QDataStream& operator>>(QDataStream&a,PExtrema&b)
{
    QString verify;
    a>>verify;
    Q_ASSERT(verify=="begin PExtrema");

    a>>b._name>>
       b._xhigh>>b._high>>b._low>>b._xlow;

    bool x;
    a>>x;
    if(x) {
        b._sxhigh=PStyle::noStyle;
    } else {
        QString idText;
        a>>idText;
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._sxhigh=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }
    a>>x;
    if(x) {
        b._shigh=PStyle::noStyle;
    } else {
        QString idText;
        a>>idText;
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._shigh=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }
    a>>x;
    if(x) {
        b._slow=PStyle::noStyle;
    } else {
        QString idText;
        a>>idText;
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._slow=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }
    a>>x;
    if(x) {
        b._sxlow=PStyle::noStyle;
    } else {
        QString idText;
        a>>idText;
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._sxlow=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }

    a>>*(PObject*)(&b);
    return a;
}

void load(QVariant r,PExtrema&b)
{
    QVariantMap v=r.toMap();
    b._name=v["_name"].toString();
    b._xhigh=v["_xhigh"].toFloat();
    b._high=v["_high"].toFloat();
    b._low=v["_low"].toFloat();
    b._xlow=v["_xlow"].toFloat();

    if(v["_sxhigh"].toString()=="NO STYLE") {
        b._sxhigh=PStyle::noStyle;
    } else {
        QString idText=v["_sxhigh"].toString();
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._sxhigh=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }

    if(v["_shigh"].toString()=="NO STYLE") {
        b._shigh=PStyle::noStyle;
    } else {
        QString idText=v["_shigh"].toString();
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._shigh=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }

    if(v["_slow"].toString()=="NO STYLE") {
        b._slow=PStyle::noStyle;
    } else {
        QString idText=v["_slow"].toString();
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._slow=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }

    if(v["_sxlow"].toString()=="NO STYLE") {
        b._sxlow=PStyle::noStyle;
    } else {
        QString idText=v["_sxlow"].toString();
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._sxlow=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }

    load(v["PObject"],*(PObject*)(&b));
}

QDataStream& operator<<(QDataStream& a,PStyle& b)
{
    a << (QString)"begin PStyle"<<
         b._bg<<b._bold<<b._fg<<b._italic<<b._linked<<b._name<<*(PObject*)(&b);
    return a;
}

QVariant save(PStyle& b)
{
    QMap<QString,QVariant> ret;
    ret.insert("_bg",b._bg);
    ret.insert("_bold",b._bold);
    ret.insert("_fg",b._fg);
    ret.insert("_italic",b._italic);
    ret.insert("_linked",b._linked);
    ret.insert("_name",b._name);
    ret.insert("PObject",save(*(PObject*)(&b)));

    return ret;
}


QDataStream& operator>>(QDataStream& a,PStyle& b)
{
    QString verify;
    a>>verify;
    Q_ASSERT(verify=="begin PStyle");

    a>>b._bg>>b._bold>>b._fg>>b._italic>>b._linked>>b._name>>*(PObject*)(&b);
    if(b._linked) {
        PStyle::_u.push_back(&b);
    }
    return a;
}

void load(QVariant v,PStyle& b)
{
    QVariantMap r=v.toMap();

    b._bg=r["_bg"].value<QColor>();
    b._bold=r["_bold"].toBool();
    b._fg=r["_fg"].value<QColor>();
    b._italic=r["_italic"].toBool();
    b._linked=r["_linked"].toBool();
    b._name=r["_name"].toString();

    load(r["PObject"],*(PObject*)(&b));

    if(b._linked) {
        PStyle::_u.push_back(&b);
    }
}

QDataStream& operator<<(QDataStream& a,PMap& b)
{
    a<<(QString)"begin PMap"<<(qint32)b._map.size()<<*(PObject*)(&b);
    for(int i=0;i<b._map.size();i++) {
        a<<(qint32)b._map.keys()[i]<<(QString)b._map.values()[i];
    }
    return a;
}

QVariant save(PMap& b)
{
    QMap<QString, QVariant> ret;
    QMap<QString, QVariant> bmap;

    for(int i=0;i<b._map.size();i++) {
        bmap.insertMulti(QString::number(b._map.keys()[i]),b._map.values()[i]);
    }

    ret.insert("_map",bmap);
    ret.insert("PObject",save(*(PObject*)(&b)));
    return ret;
}

QDataStream& operator>>(QDataStream&a,PMap&b)
{
    QString verify;
    a>>verify;
    Q_ASSERT(verify=="begin PMap");

    qint32 count;
    a>>count>>*(PObject*)(&b);
    for(int i=0;i<count;i++) {
        qint32 key;
        QString value;
        a>>key>>value;
        b._map.insertMulti(key,value);
    }
    return a;
}

void load(QVariant v,PMap&b)
{
    QVariantMap m=v.toMap();

    load(m["PObject"],*(PObject*)(&b));

    for(int i=0;i<m["_map"].toMap().count();i++) {
        b._map.insertMulti(m["_map"].toMap().keys()[i].toInt(),
                           m["_map"].toMap().values()[i].toString());
    }
}

QDataStream& operator<<(QDataStream&a,PAbstractDataItem&b)
{
    PMultiDataItem* pmdi=dynamic_cast<PMultiDataItem*>(&b);
    if(pmdi) {
        a<<(qint32)0;
        a<<*pmdi->_map;
    }

    PNumberDataItem* pndi=dynamic_cast<PNumberDataItem*>(&b);
    if(pndi) {
        a<<(qint32)1;
        a<<pndi->_format;
        a<<(bool)pndi->_extrema;
        if(pndi->_extrema) {
            a<<pndi->_extrema->idText();
        }
    }

    PTimeDataItem* ptdi=dynamic_cast<PTimeDataItem*>(&b);
    if(ptdi) {
        a<<(qint32)2;
        a<<ptdi->_format;
    }

    PDirfileDataItem* pddi=dynamic_cast<PDirfileDataItem*>(&b);
    if(pddi) {
        a<<(qint32)3;
    }

    if(!pmdi&&!pndi&&!ptdi&&!pddi) {
        a<<(qint32)12321;
        qDebug()<<"Warning: Saving PAbstractDataItem of unknown type...";
    }

    a<<(QString)"begin PAbstractDataItem"<<*(PObject*)(&b)<<
        b._caption->text()<<b._source;

    a<<((bool)(b._captionStyle==PStyle::noStyle));
    if(b._captionStyle!=PStyle::noStyle) {
        a<<b._captionStyle->idText();
    }

    a<<((bool)(b._defaultDataStyle==PStyle::noStyle));
    if(b._defaultDataStyle!=PStyle::noStyle) {
        a<<b._defaultDataStyle->idText();
    }

    return a;
}

QVariant save(PAbstractDataItem&b)
{
    QMap<QString,QVariant> ret;
    PMultiDataItem* pmdi=dynamic_cast<PMultiDataItem*>(&b);
    if(pmdi) {
        ret.insert("type","PMDI");
        ret.insert("PMDI",save(*pmdi->_map));
    }

    PNumberDataItem* pndi=dynamic_cast<PNumberDataItem*>(&b);
    if(pndi) {
        ret.insert("type","PNDI");
        ret.insert("_format",pndi->_format);
        ret.insert("_extrema",(bool)pndi->_extrema);
        if(pndi->_extrema) ret.insert("_extremaID",pndi->_extrema->idText());
    }

    PTimeDataItem* ptdi=dynamic_cast<PTimeDataItem*>(&b);
    if(ptdi) {
        ret.insert("type","PTDI");
        ret.insert("_format",ptdi->_format);
    }

    PDirfileDataItem* pddi=dynamic_cast<PDirfileDataItem*>(&b);
    if(pddi) {
        ret.insert("type","PDDI");
    }

    if(!pmdi&&!pndi&&!ptdi&&!pddi) {
        ret.insert("type","INVALID!");
        qDebug()<<"Warning: Saving PAbstractDataItem of unknown type...";
    }

    ret["PObject"]=save(*(PObject*)(&b));
    ret.insert("_caption",b._caption->text());
    ret.insert("_source",b._source);


    if(b._captionStyle!=PStyle::noStyle) {
        ret.insert("_captionStyle",b._captionStyle->idText());
    } else {
        ret.insert("_captionStyle","noStyle");
    }

    if(b._defaultDataStyle!=PStyle::noStyle) {
        ret.insert("_defaultStyle",b._defaultDataStyle->idText());
    } else {
        ret.insert("_defaultStyle","noStyle");
    }

    return ret;
}

QDataStream& operator>>(QDataStream&a,PAbstractDataItem&b)
{
    QString verify;
    a>>verify;
    Q_ASSERT(verify=="begin PAbstractDataItem");

    a>>*(PObject*)(&b);

    QString text;

    a>>text;
    b._caption->setText(text);

    a>>b._source;

    bool x;
    a>>x;
    if(x) {
        b._captionStyle=PStyle::noStyle;
    } else {
        QString idText;
        a>>idText;
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._captionStyle=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }

    a>>x;
    if(x) {
        b._defaultDataStyle=PStyle::noStyle;
    } else {
        QString idText;
        a>>idText;
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._defaultDataStyle=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }

    return a;
}

void load(QVariant v,PAbstractDataItem&b)
{
    QVariantMap m=v.toMap();

    load(m["PObject"],*(PObject*)(&b));

    b._caption->setText(m["_caption"].toString());
    b._source=m["_source"].toString();

    if(m["_captionStyle"].toString()=="noStyle") {
        b._captionStyle=PStyle::noStyle;
    } else {
        QString idText=m["_captionStyle"].toString();
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._captionStyle=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }

    if(m["_defaultStyle"].toString()=="noStyle") {
        b._defaultDataStyle=PStyle::noStyle;
    } else {
        QString idText=m["_defaultStyle"].toString();
        bool ok=0;
        for(int i=0;i<PStyle::_u.size();i++) {
            if(PStyle::_u[i]->idText()==idText) {
                ok=1;
                b._defaultDataStyle=PStyle::_u[i];
                break;
            }
        }
        Q_ASSERT(ok);
    }
}

QDataStream& operator>>(QDataStream&a,PMultiDataItem&b)
{
    return a>>*b._map;
}

void load(QVariant v,PMultiDataItem &b)
{
    load(v.toMap()["PMDI"],*b._map);
}

QDataStream& operator>>(QDataStream&a,PTimeDataItem&b)
{
    return a>>b._format;
}

void load(QVariant r, PTimeDataItem &b)
{
    b._format=r.toMap()["_format"].toString();
}

QDataStream& operator>>(QDataStream&a,PNumberDataItem&b)
{
    a>>b._format;
    bool hasExtrema;
    a>>hasExtrema;
    if(hasExtrema)
    {
        if(revId<20110806) {
            b._extrema=new PExtrema;
            a>>*b._extrema;
        } else {
            QString exIdText;
            a>>exIdText;
            bool ok=0;
            for(int i=0;i<PExtrema::_u.size();i++) {
                if(PExtrema::_u[i]->idText()==exIdText) {
                    b._extrema=PExtrema::_u[i];
                    ok=1;
                    break;
                }
            }
            Q_ASSERT(ok);
        }
    }

    return a;
}

void load(QVariant v,PNumberDataItem&b)
{
    QVariantMap m=v.toMap();
    b._format=m["_format"].toString();
    if(m["_extrema"].toBool())
    {
        QString exIdText;
        exIdText=m["_extremaID"].toString();
        bool ok=0;
        for(int i=0;i<PExtrema::_u.size();i++) {
            if(PExtrema::_u[i]->idText()==exIdText) {
                b._extrema=PExtrema::_u[i];
                ok=1;
                break;
            }
        }
        Q_ASSERT(ok);
    }
}

QDataStream& operator<<(QDataStream&a,POwlAnimation&b)
{
    a<<(QString)"begin Owl";
    return a<<b.geometry()<<*(PObject*)(&b);
}

QVariant save(POwlAnimation&b)
{
    QVariantMap map;
    map.insert("geometryX",b.geometry().x());
    map.insert("geometryY",b.geometry().y());
    map.insert("geometryWidth",b.geometry().width());
    map.insert("geometryHeight",b.geometry().height()); //our wonderful json serialzer doesn't like QRects :(
    map.insert("PObject",save(*(PObject*)(&b)));
    return map;
}

QDataStream& operator>>(QDataStream&a,POwlAnimation&b)
{
    QString verify;
    a>>verify;
    Q_ASSERT(verify=="begin Owl");

    QRect geo;
    a>>geo>>*(PObject*)(&b);
    b.setGeometry(geo);
    return a;
}

void load(QVariant v,POwlAnimation&b) {
    QVariantMap m = v.toMap();
    b.setGeometry(QRect(m["geometryX"].toInt(),m["geometryY"].toInt(),m["geometryWidth"].toInt(),m["geometryHeight"].toInt()));
    load(m["PObject"],*(PObject*)(&b));
}

QDataStream& operator<<(QDataStream&a,PBox&b)
{
    a<<(QString)"begin PBox"<<*(PObject*)(&b);
    a<<b.geometry()<<b._boxTitle<<(qint32)b._dataItems.size();
    for(int i=0;i<b._dataItems.size();i++) {
        a<<*b._dataItems[i];
    }
    return a;
}

QVariant save(PBox&b)
{
    QVariantMap map;
    map.insert("PObject",save(*(PObject*)(&b)));
    map.insert("geometryX",b.geometry().x());
    map.insert("geometryY",b.geometry().y());
    map.insert("geometryWidth",b.geometry().width());
    map.insert("geometryHeight",b.geometry().height());  //our wonderful json parser doesn't like parsing qrects
    map.insert("_boxTitle",b._boxTitle);
    if(b._pstyle!=PStyle::noStyle) {
        map.insert("_style",b._pstyle->idText());
    } else {
        map.insert("_style","noStyle");
    }

    for(int i=0;i<b._dataItems.size();i++) {
        map.insertMulti("dataItem object",save(*b._dataItems[i]));
    }
    return map;
}

QDataStream& operator>>(QDataStream&a,PBox&b)
{
    QString verify;
    a>>verify;
    Q_ASSERT(verify=="begin PBox");

    a>>*(PObject*)(&b);

    QRect geo;
    QString title;
    qint32 itemCount;
    a>>geo>>title>>itemCount;
    b.setGeometry(geo);
    b.setBoxTitle(title);

    for(int i=0;i<itemCount;i++) {
        PAbstractDataItem* padi=0;

        qint32 type;
        a>>type;
        switch(type) {
        case -1:
            padi=new PAbstractDataItem(&b,"Loaded Item");
            break;
        case 0:
            a>>*(qobject_cast<PMultiDataItem*>(padi=new PMultiDataItem(&b,"Loaded Item")));
            break;
        case 1:
            a>>*(qobject_cast<PNumberDataItem*>(padi=new PNumberDataItem(&b,"Loaded Item")));
            break;
        case 2:
            a>>*(qobject_cast<PTimeDataItem*>(padi=new PTimeDataItem(&b,"Loaded Item")));
            break;
        case 3:
            padi=new PDirfileDataItem(&b,"Loaded Item");
            break;
        default:
            qFatal("In this version of owl, only abstract, multi, number and time data items are supported. Aborting.");
            break;
        }
        a>>*padi;
        b.addProperty(padi);
    }

    return a;
}

void load(QVariant v,PBox&b)
{
    QVariantMap m=v.toMap();
    load(m["PObject"],*(PObject*)(&b));

    b.setGeometry(m["geometryX"].toInt(),m["geometryY"].toInt(),m["geometryWidth"].toInt(),m["geometryHeight"].toInt());
    b.setBoxTitle(m["_boxTitle"].toString());

    QString idText=m["_style"].toString();
    bool ok=0;
    for(int i=0;i<PStyle::_u.size();i++) {
        if(PStyle::_u[i]->idText()==idText) {
            ok=1;
            b._pstyle=PStyle::_u[i];
            break;
        }
    }
    if (!ok) {
        qDebug() << "warning: box without style";
    }

    for(int i=m.count("dataItem object")-1;i>=0;i--) {  //BUG (?) - there is no guarantee of order in maps, so you might need to change this
        PAbstractDataItem* padi=0;

        if(m.values("dataItem object")[i].toMap()["type"]=="PMDI") {
            load(m.values("dataItem object")[i],*qobject_cast<PMultiDataItem*>(padi=new PMultiDataItem(&b,"Loaded Item")));
        }
        else if(m.values("dataItem object")[i].toMap()["type"]=="PNDI") {
            load(m.values("dataItem object")[i],*qobject_cast<PNumberDataItem*>(padi=new PNumberDataItem(&b,"Loaded Item")));
        }
        else if(m.values("dataItem object")[i].toMap()["type"]=="PTDI") {
            load(m.values("dataItem object")[i],*qobject_cast<PTimeDataItem*>(padi=new PTimeDataItem(&b,"Loaded Item")));
        }
        else if(m.values("dataItem object")[i].toMap()["type"]=="PDDI") {
            load(m.values("dataItem object")[i],*qobject_cast<PDirfileDataItem*>(padi=new PDirfileDataItem(&b,"Loaded Item")));
        }
        else {
            qFatal("In this version of owl, only multi, number and time data items are supported. Aborting.");
        }
        load(m.values("dataItem object")[i],*padi);
        b.addProperty(padi);
    }
}

QDataStream& operator<<(QDataStream&a,PMainWindow&b)
{
    a<<(QString)"OWL FILE rev. 20110809";

    bool rec[10000]={0};

    a<<(qint32)PStyle::_u.size();
    for(int i=0;i<PStyle::_u.size();i++) {
        a<<*PStyle::_u[i];
    }

    QList<PExtrema*> uniques;
    for(int i=0;i<PExtrema::_u.size();i++) {
        if(!rec[PExtrema::_u[i]->id()]) {
            rec[PExtrema::_u[i]->id()]=1;
            uniques.push_back(PExtrema::_u[i]);
        }
    }
    a<<(qint32)uniques.size();
    for(int i=0;i<uniques.size();i++) {
        a<<*uniques[i];
    }

    a<<(QString)b._dirfileFilename;

    a<<1;
    a<<*(PObject*)(&b);

    a<<(qint32)b._owlList.size();
    for(int i=0;i<b._owlList.size();i++) {
        a<<*b._owlList[i];
    }

    a<<(qint32)b._pboxList.size();
    for(int i=0;i<b._pboxList.size();i++) {
        a<<*b._pboxList[i];
    }
    return a;
}

QVariant save(PMainWindow&b)
{
    QVariantMap ret;
    ret.insert("OWL FILE rev.",20120716);
    ret.insert("windowWidth", b.size().width());
    ret.insert("windowHeight", b.size().height());

    QMap<int,bool> rec;

    for(int i=0;i<PStyle::_u.size();i++) {
        ret.insertMulti("PStyle object",save(*PStyle::_u[i]));
    }

    QList<PExtrema*> uniques;
    for(int i=0;i<PExtrema::_u.size();i++) {
        if(!rec.value(PExtrema::_u[i]->id(),0)) {
            rec.insert(PExtrema::_u[i]->id(),1);
            uniques.push_back(PExtrema::_u[i]);
        }
    }

    for(int i=0;i<uniques.size();i++) {
        ret.insertMulti("PExtrema object",save(*uniques[i]));
    }

    ret.insert("PObject",save(*(PObject*)(&b)));

    for(int i=0;i<b._owlList.size();i++) {
        ret.insertMulti("Owl",save(*b._owlList[i]));
    }

    for(int i=0;i<b._pboxList.size();i++) {
        ret.insertMulti("PBox object",save(*b._pboxList[i]));
    }
    return ret;
}

QDataStream& operator>>(QDataStream&a,PMainWindow&b)
{
    QString verify;
    a>>verify;
    if(!verify.startsWith("OWL FILE rev. ")) {
        QMessageBox::critical(0,"Invalid File", "The file you are trying to load is not an owl fail. Quitting.",QMessageBox::Ok);
        qFatal("Not an owl file.");
    }
    verify.remove("OWL FILE rev. ");
    revId=verify.toInt();
    if(revId!=20110809) {
        QMessageBox::critical(0,"Unsupported OWL File","The OWL file you are trying to load is not supported by this "
                              "version of OWL. Quitting.",QMessageBox::Ok);
        qFatal("Unsupported owl file.");
    }

    qint32 ncount;

    a>>ncount;
    for(int i=0;i<ncount;i++) {
        PStyle* nee=new PStyle("Loaded PStyle");
        a>>*nee;
    }

    a>>ncount;
    for(int i=0;i<ncount;i++) {
        PExtrema* pee=new PExtrema;
        a>>*pee;
    }

    QString curfile;
    a>>curfile;
    b.curfileLogic();

    qint32 htmlInterval;
    a>>htmlInterval;

    a>>*(PObject*)(&b);

    qint32 owlCount;
    a>>owlCount;
    for(int i=0;i<owlCount;i++) {
        POwlAnimation*oa=new POwlAnimation;
        a>>*oa;
        b._mdiArea->createOwl(0,0,oa);
    }

    qint32 pboxCount;
    a>>pboxCount;
    for(int i=0;i<pboxCount;i++) {
        PBox*pbox=new PBox("Loaded Box");
        b._currentObject=pbox;
        a>>*pbox;
        b._mdiArea->createPBox(0,0,pbox);
        QObject::connect(pbox,SIGNAL(activated()),&b,SLOT(uiLogic()));


    }
    for(int i=0;i<b._pboxList.count();i++) {
        for(int j=0;j<b._pboxList[i]->_dataItems.size();j++) {
            b.newLabelLogic(b._pboxList[i]->_dataItems[j]);
        }
    }
    for(int i=0;i<PExtrema::_u.size();i++) {
        b.recognizeExtrema(PExtrema::_u[i]);
    }
    return a;
}

void load(QVariant v,PMainWindow&b)
{
    PObject::isLoading=1;
    QVariantMap m=v.toMap();
    if(m["OWL FILE rev."].toInt()>20120716) {
        QMessageBox::critical(0,"Unsupported OWL File","The OWL file you are trying to load is too new for this "
                              "version of OWL. Quitting.",QMessageBox::Ok);
        qFatal("The OWL file you are trying to load is too new for this version of OWL. Quitting.");
    }

    if(m["OWL FILE rev."].toInt()>=20120716) {
        int windowWidth = m["windowWidth"].toInt();
        int windowHeight = m["windowHeight"].toInt();

        b.resize(windowWidth, windowHeight);
    }

    for(int i=0;i<m.count("PStyle object");i++) {
        PStyle* nee=new PStyle("Loaded PStyle");
        load(m.values("PStyle object")[i],*nee);
    }

    for(int i=0;i<m.count("PExtrema object");i++) {
        PExtrema* pee=new PExtrema;
        load(m.values("PExtrema object")[i],*pee);
    }

    load(m["PObject"],*(PObject*)(&b));

    for(int i=0;i<m.count("Owl");i++) {
        POwlAnimation*oa=new POwlAnimation;
        load(m.values("Owl")[i],*oa);
        b._mdiArea->createOwl(0,0,oa);
    }

    for(int i=0;i<m.count("PBox object");i++) {
        PBox*pbox=new PBox("Loaded Box");
        b._currentObject=pbox;
        load(m.values("PBox object")[i],*pbox);
        b._mdiArea->createPBox(0,0,pbox);
        QObject::connect(pbox,SIGNAL(activated()),&b,SLOT(uiLogic()));

    }
    for(int i=0;i<b._pboxList.count();i++) {
        for(int j=0;j<b._pboxList[i]->_dataItems.size();j++) {
            b.newLabelLogic(b._pboxList[i]->_dataItems[j]);
        }
    }
    for(int i=0;i<PExtrema::_u.size();i++) {
        b.recognizeExtrema(PExtrema::_u[i]);
    }
    PObject::isLoading=0;
}


