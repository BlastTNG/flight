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

#include "PDotPal.h"
#include "PExtrema.h"
#include "PMainWindow.h"
#include "PDirfileDataItem.h"
#include "PTimeDataItem.h"
#include <QFile>
#include <QDomElement>
#include <QMessageBox>

PDotPal::PDotPal(QString filen)
{
    QMessageBox::information(0,"Notice","Opening .pal files is not fully supported. Not all data will be imported.");
    QDomDocument doc("mydocument");
    QFile file(filen);
    if (!file.open(QIODevice::ReadOnly))
        return;
    if (!doc.setContent(&file)) {
        file.close();
        return;
    }
    file.close();

    QDomElement docElem = doc.documentElement();

    QDomNode n = docElem.firstChild();
    while(!n.isNull()) {
        QDomElement e = n.toElement(); // try to convert the node to an element.

        /////////////////////////////////////////////////////////////////////////////////
        if(!e.isNull()&&e.tagName().toUpper()=="SETTINGS") {
            QDomNode nn=e.childNodes().at(0);
            while(!nn.isNull()) {
                QDomElement ee=nn.toElement();

                /////////////////////////////////////////////////////////////////////////////////
                if(!ee.isNull()&&ee.tagName().toUpper()=="TEXTSTYLE") {
                    QString sName=ee.attribute("name");
                    QColor sFGColor=ee.attribute("colour","black");
                    QColor sBGColor=ee.attribute("backcolour","white");
                    if(sName=="xhiwarn") sName="xhigh";
                    if(sName=="hiwarn") sName="high";
                    if(sName=="lowarn") sName="low";
                    if(sName=="xlowarn") sName="xlow";
                    bool isBold=(ee.attribute("bold","false")=="true");
                    new PStyle(sName,isBold,0,sBGColor,sFGColor,1);
                }

                /////////////////////////////////////////////////////////////////////////////////
                if(!ee.isNull()&&ee.tagName().toUpper()=="EXTREMA") {
                    QString eName=ee.attribute("name");
                    double eHigh=1.0,eLow=-1.0,eXHigh=2.0,eXLow=-2.0;

                    QDomNode nnn=ee.childNodes().at(0);
                    while(!nnn.isNull()) {
                        QDomElement eee=nnn.toElement();

                        /////////////////////////////////////////////////////////////////////////////////
                        if(!eee.isNull()&&eee.tagName().toUpper()=="HI") {
                            eHigh=eee.attribute("value").toDouble();
                        /////////////////////////////////////////////////////////////////////////////////
                        } else if(!eee.isNull()&&eee.tagName().toUpper()=="XHI") {
                            eXHigh=eee.attribute("value").toDouble();
                        /////////////////////////////////////////////////////////////////////////////////
                        } else if(!eee.isNull()&&eee.tagName().toUpper()=="LO") {
                            eLow=eee.attribute("value").toDouble();
                        /////////////////////////////////////////////////////////////////////////////////
                        } else if(!eee.isNull()&&eee.tagName().toUpper()=="XLO") {
                            eXLow=eee.attribute("value").toDouble();
                        }

                        nnn=nnn.nextSibling();
                    }
                    PExtrema*p=new PExtrema;
                    p->setName(eName);
                    p->_low=eLow;
                    p->_high=eHigh;
                    p->_xlow=eXLow;
                    p->_xhigh=eXHigh;
                }
                nn=nn.nextSibling();
            }
        /////////////////////////////////////////////////////////////////////////////////
        } else if(!e.isNull()&&e.tagName().toUpper()=="BOX") {
            _pbox.push_back(new PBox("DOTPAL BOX"));
            _pbox.back()->setBoxTitle(e.attribute("caption"),1);
//            _pbox.back()->setColour(e.attribute("colour"),1);

            QDomNode nn=e.childNodes().at(0);
            while(!nn.isNull()) {
                QDomElement ee=nn.toElement();

                /////////////////////////////////////////////////////////////////////////////////
                if(!ee.isNull()&&ee.tagName().toUpper()=="MULTI") {
                    PMultiDataItem* pmdi=new PMultiDataItem(_pbox.back(),"DOTPAL PMDI");
                    pmdi->setCaption(ee.attribute("caption","DOTPAL_NOCAPTION"),1);

                    QDomNode nnn=ee.childNodes().at(0);
                    while(!nnn.isNull()) {
                        QDomElement eee=nnn.toElement();

                        /////////////////////////////////////////////////////////////////////////////////
                        if(!eee.isNull()&&eee.tagName().toUpper()=="DATUM") {
                            pmdi->setSource(eee.attribute("src"),1);
                            QDomNode nnnn=eee.childNodes().at(0);
                            while(!nnnn.isNull()) {
                                QDomElement eeee=nnnn.toElement();

                                /////////////////////////////////////////////////////////////////////////////////
                                if(!eeee.isNull()&&eeee.tagName().toUpper()=="WORD") {
                                    QString style=eeee.attribute("textstyle","defbox");
                                    QString idText="INVALID!!!";
                                    for(int i=0;i<PStyle::_u.size();i++) {
                                        if(style==PStyle::_u[i]->name()) {
                                            idText=PStyle::_u[i]->idText();
                                        }
                                    }
                                    pmdi->_map->set(
                                                eeee.attribute("value").toInt(),
                                                eeee.attribute("caption")+
                                                " [style=\""+style+"\" "+idText+"]");
                                }
                                nnnn=nnnn.nextSibling();
                            }
                        }

                        nnn=nnn.nextSibling();
                    }
                    _pbox.back()->addProperty(pmdi);
                /////////////////////////////////////////////////////////////////////////////////
                } else if(!ee.isNull()&&ee.tagName().toUpper()=="NUMBER") {
                    PNumberDataItem* pndi=new PNumberDataItem(_pbox.back(),"DOTPAL PNDI");
                    _pbox.back()->addProperty(pndi);
                    pndi->setCaption(ee.attribute("caption"),1);

                    QDomNode nnn=ee.childNodes().at(0);
                    while(!nnn.isNull()) {
                        QDomElement eee=nnn.toElement();

                        /////////////////////////////////////////////////////////////////////////////////
                        if(!eee.isNull()&&eee.tagName().toUpper()=="DATUM") {
                            pndi->setSource(eee.attribute("src"),1);
                            pndi->setFormat(eee.attribute("format"),1);
                            QString extremaName=eee.attribute("extrema","unknown");
                            if(extremaName!="unknown") {
                                for(int i=0;i<PExtrema::_u.size();i++) {
                                    if(PExtrema::_u[i]->name()==extremaName) {
                                        pndi->_extrema=PExtrema::_u[i];
                                    }
                                }
                            }
                        }

                        nnn=nnn.nextSibling();
                    }
                /////////////////////////////////////////////////////////////////////////////////
                } else if(!ee.isNull()&&ee.tagName().toUpper()=="CURDIR") {
                    PDirfileDataItem* pddi=new PDirfileDataItem(_pbox.back(),"DOTPAL PDDI");
                    _pbox.back()->addProperty(pddi);
                    pddi->setCaption(ee.attribute("caption"),1);
                /////////////////////////////////////////////////////////////////////////////////
                } else if(!ee.isNull()&&ee.tagName().toUpper()=="DATETIME") {
                    PTimeDataItem* ptdi=new PTimeDataItem(_pbox.back(),"DOTPAL PTDI");
                    _pbox.back()->addProperty(ptdi);;
                    ptdi->setCaption(ee.attribute("caption"),1);

                    QDomNode nnn=ee.childNodes().at(0);
                    while(!nnn.isNull()) {
                        QDomElement eee=nnn.toElement();

                        /////////////////////////////////////////////////////////////////////////////////
                        if(!eee.isNull()&&eee.tagName().toUpper()=="DATUM") {
                            ptdi->setSource(eee.attribute("src"),1);
                            ptdi->setFormat(eee.attribute("format"),1);
                        }

                        nnn=nnn.nextSibling();
                    }
                }
                nn=nn.nextSibling();
            }
        }


        n = n.nextSibling();
    }

    // Here we append a new element to the end of the document
    QDomElement elem = doc.createElement("img");
    elem.setAttribute("src", "myimage.png");
    docElem.appendChild(elem);

    for(int i=0;i<PExtrema::_u.size();i++) {
        PMainWindow::me->recognizeExtrema(PExtrema::_u[i]);
    }
}
