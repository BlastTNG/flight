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

QDataStream& operator<<(QDataStream& a,PObject& b)
{
    return a << (QString)"begin PObject"<<
                b._id<<b._lastId;
}


QDataStream& operator>>(QDataStream& a,PObject& b)
{
    QString verify;
    a>>verify;
    Q_ASSERT(verify=="begin PObject");

    return a>>b._id>>b._lastId;
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

QDataStream& operator<<(QDataStream& a,PStyle& b)
{
    a << (QString)"begin PStyle"<<
         b._bg<<b._bold<<b._fg<<b._italic<<b._linked<<b._name<<*(PObject*)(&b);
    return a;
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

QDataStream& operator<<(QDataStream& a,PMap& b)
{
    a<<(QString)"begin PMap"<<(qint32)b._map.size()<<*(PObject*)(&b);
    for(int i=0;i<b._map.size();i++) {
        a<<(qint32)b._map.keys()[i]<<(QString)b._map.values()[i];
    }
    return a;
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

QDataStream& operator>>(QDataStream&a,PMultiDataItem&b)
{
    return a>>*b._map;
}

QDataStream& operator>>(QDataStream&a,PTimeDataItem&b)
{
    return a>>b._format;
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

QDataStream& operator<<(QDataStream&a,POwlAnimation&b)
{
    a<<(QString)"begin Owl";
    return a<<b.geometry()<<*(PObject*)(&b);
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

QDataStream& operator<<(QDataStream&a,PBox&b)
{
    a<<(QString)"begin PBox"<<*(PObject*)(&b);
    a<<b.geometry()<<b._boxTitle<<(qint32)b._dataItems.size();
    for(int i=0;i<b._dataItems.size();i++) {
        a<<*b._dataItems[i];
    }
    return a;
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
    b.curfileLogic(curfile);

    qint32 htmlInterval;
    a>>htmlInterval;

    a>>*(PObject*)(&b);

    qint32 owlCount;
    a>>owlCount;
    for(int i=0;i<owlCount;i++) {
        b._mdiArea->createOwl();
        a>>*b._owlList.back();
    }

    qint32 pboxCount;
    a>>pboxCount;
    for(int i=0;i<pboxCount;i++) {
        PBox*pbox=new PBox("Loaded Box");
        b._currentObject=pbox;
        a>>*pbox;
        b._mdiArea->createPBox(0,0,pbox);
        QObject::connect(pbox,SIGNAL(activated()),&b,SLOT(uiLogic()));
        QObject::connect(pbox,SIGNAL(newChild(PAbstractDataItem*)),
                &b,SLOT(newLabelLogic(PAbstractDataItem*)));

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

