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
#include "PExtremaDataItem.h"
#include "PDirfileDataItem.h"
#include "PBox.h"
#include "PExtrema.h"
#include "PMainWindow.h"
#include "PMap.h"
#include "PMdiArea.h"
#include "PMdiArea.h"
#include "PBitMultiDataItem.h"
#include "PMultiDataItem.h"
#include "PNumberDataItem.h"
#include "PTimeDataItem.h"
#include "PObject.h"
#include <QTimer>
#include <QDebug>
#include <QMessageBox>

#define OLDEST_SUPPORTED_OWL_REVISION 20140928
#define LATEST_OWL_REVISION 20140928
int revId=-1;
int _lastId=0;  //in order to not break anything

QVariant save(PObject& b)
{
    QMap<QString,QVariant> ret;
    ret.insert("_id",QString::number(b._id));
    return ret;
}

void load(QVariant s,PObject& b)
{
    b._id=s.toMap()["_id"].toInt();
    PObject::_u[b._id]=&b;
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

QVariant save(PMap& b)
{
    QMap<QString, QVariant> ret;
    QMap<QString, QVariant> bmap;

    for(int i=0;i<b._map.size();i++) {
        bmap.insertMulti(QString::number(b._map.keys()[i]), b._map.values()[i]);
    }

    ret.insert("_map",bmap);
    ret.insert("PObject",save(*(PObject*)(&b)));
    return ret;
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

QVariant save(PAbstractDataItem&b)
{
    QMap<QString,QVariant> ret;
    PBitMultiDataItem* pbdi=dynamic_cast<PBitMultiDataItem*>(&b);
    if(pbdi) {
        ret.insert("type","PBDI");
        ret.insert("_highWord",pbdi->_highWord);
        ret.insert("_lowWord",pbdi->_lowWord);
        ret.insert("_nBits",pbdi->_nBits);
    }

    PMultiDataItem* pmdi=dynamic_cast<PMultiDataItem*>(&b);
    if(pmdi) {
        ret.insert("type","PMDI");
        ret.insert("PMDI",save(*pmdi->_map));
    }

    PNumberDataItem* pndi=dynamic_cast<PNumberDataItem*>(&b);
    if(pndi) {
        ret.insert("type","PNDI");
        ret.insert("_format",pndi->_format);
    }

    PTimeDataItem* ptdi=dynamic_cast<PTimeDataItem*>(&b);
    if(ptdi) {
        ret.insert("type","PTDI");
        ret.insert("_format",ptdi->_format);
    }

    PExtremaDataItem *pedi=dynamic_cast<PExtremaDataItem*>(&b);
    if (pedi) {
        ret.insert("_extrema",(bool)pedi->_extrema);
        if(pedi->_extrema) ret.insert("_extremaID",pedi->_extrema->idText());
    }

    PDirfileDataItem* pddi=dynamic_cast<PDirfileDataItem*>(&b);
    if(pddi) {
        ret.insert("type","PDDI");
    }

    if(!pbdi&&!pmdi&&!pndi&&!ptdi&&!pddi) {
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

void load(QVariant v, PExtremaDataItem &b)
{
    QVariantMap m=v.toMap();
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

void load(QVariant v,PBitMultiDataItem &b)
{
    QVariantMap m=v.toMap();
    b._highWord=m["_highWord"].toString();
    b._lowWord=m["_lowWord"].toString();
    b._nBits=m["_nBits"].toInt();
}

void load(QVariant v,PMultiDataItem &b)
{
    load(v.toMap()["PMDI"],*b._map);
}

void load(QVariant r, PTimeDataItem &b)
{
    b._format=r.toMap()["_format"].toString();
}

void load(QVariant v,PNumberDataItem&b)
{
    QVariantMap m=v.toMap();
    b._format=m["_format"].toString();
}

QVariant save(POwlAnimation&b)
{
    QVariantMap map;

    double scale = b.fontMetrics().height()/18.0;
    map.insert("geometryX",int(b.geometry().x()));
    map.insert("geometryY",int(b.geometry().y()));
    map.insert("geometryWidth",int(b.geometry().width()));
    map.insert("geometryHeight",int(b.geometry().height())); //our wonderful json serialzer doesn't like QRects :(

    // new geometry: in font-height cells
    scale = b.fontMetrics().height();
    map.insert("cellX",int(b.geometry().x()/scale+0.49));
    map.insert("cellY",int(b.geometry().y()/scale+0.49));
    map.insert("cellWidth",int(b.geometry().width()/scale+0.49));
    map.insert("cellHeight",int(b.geometry().height()/scale+0.49));

    map.insert("PObject",save(*(PObject*)(&b)));
    return map;
}
void load(QVariant v,POwlAnimation&b) {
    QVariantMap m = v.toMap();

    double scale;
    double w, h, x, y;

    w = m["cellWidth"].toDouble();

    if (w>0) {
      scale = b.fontMetrics().height();
      w *= scale;
      h = m["cellHeight"].toDouble() * scale;
      x = m["cellX"].toDouble() * scale;
      y = m["cellY"].toDouble() * scale;
    } else {
      scale = b.fontMetrics().height()/18.0;
      w = m["geometryWidth"].toDouble() * scale;
      h = m["geometryHeight"].toDouble() * scale;
      x = m["geometryX"].toDouble() * scale;
      y = m["geometryY"].toDouble() * scale;
    }

    b.setGeometry(QRect(x, y, w, h));

    load(m["PObject"],*(PObject*)(&b));
}

QVariant save(PBox&b)
{
    QVariantMap map;
    QVariantList dataItems;
    for(int i=0;i<b._dataItems.size();i++) {
        dataItems.push_back(save(*b._dataItems[i]));
    }

    double scale = b.fontMetrics().height()/18.0;

    map.insert("PObject",save(*(PObject*)(&b)));
    map.insert("geometryX",int(b.geometry().x()));
    map.insert("geometryY",int(b.geometry().y()));
    map.insert("geometryWidth",int(b.geometry().width()));
    map.insert("geometryHeight",int(b.geometry().height()));  //our wonderful json parser doesn't like parsing qrects

    // new geometry: in font-height cells
    scale = b.fontMetrics().height();
    map.insert("cellX",int(b.geometry().x()/scale+0.49));
    map.insert("cellY",int(b.geometry().y()/scale+0.49));
    map.insert("cellWidth",int(b.geometry().width()/scale+0.49));
    map.insert("cellHeight",int(b.geometry().height()/scale+0.49));

    map.insert("_boxTitle",b._boxTitle);
    if(b._pstyle!=PStyle::noStyle) {
        map.insert("_style",b._pstyle->idText());
    } else {
        map.insert("_style","noStyle");
    }
    map.insert("dataItems", dataItems);
    return map;
}

void load(QVariant v,PBox&b)
{
    QVariantMap m=v.toMap();

    double scale;
    double w, h, x, y;

    w = m["cellWidth"].toDouble();

    if (w>0) {
      scale = b.fontMetrics().height();
      w *= scale;
      h = m["cellHeight"].toDouble() * scale;
      x = m["cellX"].toDouble() * scale;
      y = m["cellY"].toDouble() * scale;
    } else {
      scale = b.fontMetrics().height()/18.0;
      w = m["geometryWidth"].toDouble() * scale;
      h = m["geometryHeight"].toDouble() * scale;
      x = m["geometryX"].toDouble() * scale;
      y = m["geometryY"].toDouble() * scale;
    }

    b.setGeometry(QRect(x, y, w, h));


    load(m["PObject"],*(PObject*)(&b));

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

    QVariantList dataItems = m.value("dataItems").toList();
    for(int i = 0; i < dataItems.length(); ++i) {
        PAbstractDataItem* padi=0;

        if(dataItems[i].toMap()["type"]=="PMDI") {
            load(dataItems[i],*qobject_cast<PMultiDataItem*>(padi=new PMultiDataItem(&b,"Loaded Item")));
        } else if(dataItems[i].toMap()["type"]=="PBDI") {
            load(dataItems[i],*qobject_cast<PBitMultiDataItem*>(padi=new PBitMultiDataItem(&b,"Loaded Item")));
        } else if(dataItems[i].toMap()["type"]=="PNDI") {
            load(dataItems[i],*qobject_cast<PNumberDataItem*>(padi=new PNumberDataItem(&b,"Loaded Item")));
        } else if(dataItems[i].toMap()["type"]=="PTDI") {
            load(dataItems[i],*qobject_cast<PTimeDataItem*>(padi=new PTimeDataItem(&b,"Loaded Item")));
        } else if(dataItems[i].toMap()["type"]=="PDDI") {
            load(dataItems[i],*qobject_cast<PDirfileDataItem*>(padi=new PDirfileDataItem(&b,"Loaded Item")));
        } else {
            qFatal("Unsupported data type in .owl file.  Aborting.\n");
        }
        PExtremaDataItem *pedi=dynamic_cast<PExtremaDataItem*>(padi);
        if (pedi)
          load(dataItems[i], *pedi);
        load(dataItems[i], *padi);
        b.addProperty(padi);
    }
}

QVariant save(PMainWindow&b)
{
    QVariantMap root;
    QMap<int,bool> allExtremas;
    QVariantList styles;
    QVariantList uniqueExtremas;
    QVariantList owls;
    QVariantList boxes;

    for (int i=0;i<PStyle::_u.size();i++) {
        styles.push_back(save(*PStyle::_u[i]));
    }

    // There can be duplicate extremas, so filter them, and save
    // all the unique ones to uniqueExtremas
    for(int i=0;i<PExtrema::_u.size();i++) {
        if(!allExtremas.value(PExtrema::_u[i]->id(),0)) {
            allExtremas.insert(PExtrema::_u[i]->id(),1);
            uniqueExtremas.push_back(save(*PExtrema::_u[i]));
        }
    }

    for(int i=0;i<b._owlList.size();i++) {
        owls.push_back(save(*b._owlList[i]));
    }

    for(int i=0;i<b._pboxList.size();i++) {
        boxes.push_back(save(*b._pboxList[i]));
    }

    root.insert("OWL FILE rev.",QString::number(LATEST_OWL_REVISION));
    root.insert("PObject",save(*(PObject*)(&b))); // Add the root ID
    root.insert("linkSelected", b._link);
    root.insert("linkNames", b.linkNames());
    root.insert("windowWidth", b.size().width());
    root.insert("windowHeight", b.size().height());
    root.insert("windowCellWidth", int(double(b.size().width())/double(b.fontMetrics().height())+0.49));
    root.insert("windowCellHeight", b.size().height()/b.fontMetrics().height());
    root.insert("styles", styles);
    root.insert("owls", owls);
    root.insert("boxes", boxes);
    root.insert("extremas", uniqueExtremas);
#if QT_VERSION >= 0x050300
    root.insert("webPort", QString(b.webPort()));
#else
    root.insert("webPort", "0");
#endif
    return root;
}

void load(QVariant v,PMainWindow&b)
{
    PObject::isLoading=1;
    QVariantMap m=v.toMap();
    int rev = m["OWL FILE rev."].toInt();

    if(rev < OLDEST_SUPPORTED_OWL_REVISION) {
        QMessageBox::critical(0,"Unsupported OWL File","The OWL file you are "
            "trying to load is too old. Please open the transitional version of "
            "Owl 4 and save it in the new format.",
            QMessageBox::Ok);
        qFatal("Unknown file version.");
    } else if(rev > LATEST_OWL_REVISION) {
        QMessageBox::critical(0,"Unsupported OWL File","The OWL file you are "
            "trying to load is too new. Update your version of Owl.",
            QMessageBox::Ok);
        qFatal("Unknown file version.");
    }

    QVariantList styles = m.value("styles").toList();
    QVariantList extremas = m.value("extremas").toList();
    QVariantList owls = m.value("owls").toList();
    QVariantList boxes = m.value("boxes").toList();
    QVariant super = m.value("PObject");
    QVariant link = m.value("linkSelected");
    QStringList linkNames = m.value("linkNames").toStringList();
    QVariant webPort = m.value("webPort");

    int wW, wH;
    wW = m.value("windowCellWidth").toInt() * b.fontMetrics().height();

    if (wW == 0) {
      wW = m.value("windowWidth").toInt();
      wH = m.value("windowHeight").toInt();
    } else {
      wH = m.value("windowCellHeight").toInt() * b.fontMetrics().height();
    }

    b.setLink(link.toInt(), linkNames);

    b.resize(wW, wH);

    b._mdiArea->setFont(b.font());
    b._mdiArea->set_H(b._mdiArea->fontMetrics().height());
    for(int i = 0; i < styles.length(); ++i) {
        PStyle* nee=new PStyle("Loaded PStyle");
        load(styles[i],*nee);
    }

    for(int i = 0; i < extremas.length(); ++i) {
        PExtrema* pee=new PExtrema;
        load(extremas[i],*pee);
    }

    load(super,*(PObject*)(&b));

    for(int i = 0; i < owls.length(); ++i) {
        POwlAnimation*oa=new POwlAnimation(b.fontMetrics().height());
        oa->setFont(b.font());

        load(owls[i],*oa);
        b._mdiArea->createOwl(0,0,oa);
    }

    for(int i = 0; i < boxes.length(); ++i) {
        PBox*pbox=new PBox("Loaded Box", b.font());
        pbox->setFont(b.font());
        b._currentObject=pbox;
        load(boxes[i],*pbox);
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

#if QT_VERSION >= 0x050300
    b.setWebPort(webPort.isValid() ? webPort.toInt() : -1);
#endif

    PObject::isLoading=0;
}


