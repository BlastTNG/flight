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

#include <stdlib.h>
#include "PMainWindow.h"
#include "PMdiArea.h"
#include "PTimeDataItem.h"
#include "PDirfileDataItem.h"
#include "PDotPal.h"
#include "PStyle.h"
#include "PWebServerInfo.h"
#include "POwlAnimation.h"

#include "ui_PMainWindow.h"

#include <QTextEdit>
#include <QColorDialog>
#include <QTextStream>
#include <QTimer>
#include <QCompleter>
#include <QFile>
#include <QInputDialog>
#include <QMessageBox>
#include <QColorDialog>
#include <QFileDialog>
#include <QDebug>
#include <QDialogButtonBox>
#include <QFileSystemModel>
#ifdef __APPLE__
#include <python2.6/Python.h>
#else
#include <python2.7/Python.h>   //you may need to change this
#endif
#include <qjson/parser.h>
#include <qjson/serializer.h>

PMainWindow* PMainWindow::me=0;

QString PMainWindow::key;

static QString _filename;

PMainWindow::PMainWindow(QString file, QWidget *parent) :
    QMainWindow(parent),
    PObject(0),
    _currentObject(0),
    _dirfile(0),
    _scrollArea(new QScrollArea()),
    styleVersion(0),
    layoutVersion(0),
    _server(0),
    _deleteScheduled(0),
    ui(new Ui::PMainWindow)
{

    _settings = new QSettings("UToronto", "owl");
    PStyle::noStyle = PStyle::noStyle?PStyle::noStyle:new PStyle("No style");
    me=this;
    ui->setupUi(this);
    ui->comboBox->addItem("Owl "+idText());
    _mdiArea=new PMdiArea;
    _scrollArea->setParent(centralWidget());
    _scrollArea->setAutoFillBackground(1);
    QPalette p=_scrollArea->palette();
    p.setColor(_scrollArea->backgroundRole(),"white");
    _scrollArea->setPalette(p);
    _mdiArea->setMinimumSize(1600,1200);
    _scrollArea->setWidget(_mdiArea);
    _mdiArea->adjustSize();
    centralWidget()->layout()->addWidget(_scrollArea);
    connect(_mdiArea,SIGNAL(newBox(PBox*)),this,SLOT(uiLogic()));
    connect(ui->comboBox,SIGNAL(activated(int)),this,SLOT(uiLogic()));
    connect(ui->actionSave,SIGNAL(activated()),this,SLOT(owlSave()));
    connect(ui->actionSaveAs,SIGNAL(activated()),this,SLOT(owlSaveAs()));
    connect(ui->actionLoad,SIGNAL(activated()),this,SLOT(owlLoad()));
    connect(PStyleNotifier::me,SIGNAL(change()),this,SLOT(currowLogic()));
    connect(ui->toolButtonHelp,SIGNAL(clicked()),this,SLOT(webServerHelp()));
    connect(ui->actionHelp,SIGNAL(activated()),this,SLOT(readmeHelp()));
    connect(_mdiArea,SIGNAL(newOwl(POwlAnimation*)),this,SLOT(addOwl()));
    QFileSystemModel* fsm=new QFileSystemModel();
    fsm->setRootPath(QDir::currentPath());
    ui->lineEditHighGain->setCompleter(new QCompleter(fsm));
    ui->lineEditTDRSSOmni->setCompleter(new QCompleter(fsm));
    ui->lineEditLOS->setCompleter(new QCompleter(fsm));
    ui->lineEditIridum->setCompleter(new QCompleter(fsm));
    ui->lineEditHtmlPath->setText(key);
    if (!key.isEmpty()) {
      ui->checkboxWeb_server_on->setChecked(true);
    }

    curfileLogic();

    _currowStyle=PStyle::noStyle;

    uiLogic();

    _ut=new QTimer(this);
    _ut->setInterval(500);
    connect(_ut,SIGNAL(timeout()),this,SLOT(gdUpdate()));
    _ut->start();

    activate();

    setMinimumSize(1,1);

    ui->label_kst->hide();
    ui->pushButton_kst->hide();
    connect(ui->pushButton_kst,SIGNAL(clicked()),this,SLOT(showInKst()));

    setWindowTitle(_WINDOW_TITLE_);

    if (file == "__lastfile") {
        file = _settings->value("filename").toString();
    }

    if (_settings->value("hideWeb", false).toBool()) {
        ui->actionWeb_Server->setChecked(false);
        ui->dockWeb_Server->hide();
    }
    if (_settings->value("hideInsert", false).toBool()) {
      ui->actionInsert->setChecked(false);
      ui->dockInsert->hide();
    }
    if (_settings->value("hideConfig", false).toBool()) {
      ui->actionConfigure->setChecked(false);
      ui->dockConfigure->hide();
    }
    if (_settings->value("hideLink", false).toBool()) {
      ui->actionLink->setChecked(false);
      ui->dockLink->hide();
    }

    if(file.size()) {
        owlLoad(file);
        _filename = file;
    }

    if(!_deleteScheduled&&!PObject::isLoading) show();

}

#define reconnect(a,b,c,d) \
    disconnect(a,b,c,d); \
    connect(a,b,c,d)

PMainWindow::~PMainWindow()
{

    _settings->setValue("hideWeb",!ui->dockWeb_Server->isVisible());
    _settings->setValue("hideInsert",!ui->dockInsert->isVisible());
    _settings->setValue("hideConfig",!ui->dockConfigure->isVisible());
    _settings->setValue("hideLink",!ui->dockLink->isVisible());

    while(_pboxList.size()) {
        delete _pboxList.takeFirst();
    }
    while(_owlList.size()) {
        delete _owlList.takeFirst();
    }
}

void PMainWindow::closeEvent(QCloseEvent* e)
{
    e->setAccepted(QMessageBox::question(this,"Really Quit?","If you quit, you will lose all unsaved data. Quit anyway?",QMessageBox::Yes,QMessageBox::No)==QMessageBox::Yes);
}

void PMainWindow::readmeHelp()
{
    QFile readme(":/client/README.txt");
    readme.open(QFile::ReadOnly);

    QDialog x;
    QTextEdit te(&x);
    te.setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    te.setWordWrapMode(QTextOption::NoWrap);
    te.setFontFamily("monospace");
    te.setText(readme.readAll());
    te.setReadOnly(1);
    QVBoxLayout hb(&x);
    hb.addWidget(&te);
    QDialogButtonBox bb(QDialogButtonBox::Close);
    connect(&bb,SIGNAL(accepted()),&x,SLOT(close()));
    connect(&bb,SIGNAL(rejected()),&x,SLOT(close()));
    hb.addWidget(&bb);
    x.setWindowTitle("README");
    x.show();
    x.adjustSize();
    x.setGeometry(x.x(),x.y(),680,480);

    x.exec();
}

void PMainWindow::webServerHelp()
{
    PWebServerInfo wsi;
    wsi.exec();
}

void PMainWindow::hideEverything()
{
    setUpdatesEnabled(0);
    ui->pstyleCaption->hide();
    disconnect(ui->pstyleCaption,0,0,0);
    ui->pstyleData->hide();
    disconnect(ui->pstyleData,0,0,0);
    ui->labelCaption->hide();
    ui->lineEditCaption->hide();
    disconnect(ui->lineEditCaption,0,0,0);
    //ui->labelColour->hide();
    ui->labelSource->hide();
    ui->lineEditSource->hide();
    disconnect(ui->lineEditSource,0,0,0);
    ui->labelType->hide();
    ui->comboBoxType->hide();
    disconnect(ui->comboBoxType,0,0,0);
    ui->labelFormat_multi->hide();
    ui->tableWidgetFormat->hide();
    disconnect(ui->tableWidgetFormat,0,0,0);
    //ui->label_colour2->hide();
    ui->pushButtonAddFormat->hide();
    disconnect(ui->pushButtonAddFormat,0,0,0);
    ui->pushButtonDelFormat->hide();
    disconnect(ui->pushButtonDelFormat,0,0,0);
    ui->labelFormat_num->hide();
    ui->lineEditFormat->hide();
    disconnect(ui->lineEditFormat,0,0,0);
    ui->labelExtrema->hide();
    ui->comboBoxExtrema->hide();
    disconnect(ui->comboBoxExtrema,0,0,0);
    ui->labelName->hide();
    ui->lineEditExtremaName->hide();
    disconnect(ui->lineEditExtremaName,0,0,0);
    ui->labelXHigh->hide();
    ui->doubleSpinBoxXHigh->hide();
    disconnect(ui->doubleSpinBoxXHigh,0,0,0);
    ui->labelHigh->hide();
    ui->doubleSpinBoxHigh->hide();
    disconnect(ui->doubleSpinBoxHigh,0,0,0);
    ui->labelLow->hide();
    ui->doubleSpinBoxLow->hide();
    disconnect(ui->doubleSpinBoxLow,0,0,0);
    ui->labelXLow->hide();
    ui->doubleSpinBoxXLow->hide();
    disconnect(ui->doubleSpinBoxXLow,0,0,0);

    ui->labelCurRow->hide();
    disconnect(ui->labelCurRow,0,0,0);
    ui->labelRemove->hide();
    ui->pushButtonRemove->hide();
    disconnect(ui->pushButtonRemove,0,0,0);

    ui->pstyleXHigh->hide();
    disconnect(ui->pstyleXHigh,0,0,0);
    ui->pstyleHigh->hide();
    disconnect(ui->pstyleHigh,0,0,0);
    ui->pstyleLow->hide();
    disconnect(ui->pstyleLow,0,0,0);
    ui->pstyleXLow->hide();
    disconnect(ui->pstyleXLow,0,0,0);
    ui->pstyleSelected->hide();
    disconnect(ui->pstyleSelected,0,0,0);
    setUpdatesEnabled(1);
}

void PMainWindow::addPBox()
{
    setUpdatesEnabled(0);
    ui->comboBox->addItem(_pboxList.back()->boxTitle()+" "+_pboxList.back()->idText());
    connect(_pboxList.back(),SIGNAL(activated()),this,SLOT(uiLogic()));
    connect(_pboxList.back(),SIGNAL(newChild(PAbstractDataItem*)),
            this,SLOT(newLabelLogic(PAbstractDataItem*)));
    _currentObject=_pboxList.back();
    ui->comboBox->setCurrentIndex(ui->comboBox->count()-1);

    ui->pstyleCaption->setWidgetStyleRef(_pboxList.back()->_pstyle);
    ui->pushButtonRemove->show();
    ui->labelRemove->show();
    connect(ui->pushButtonRemove,SIGNAL(clicked()),_pboxList.back(),SLOT(deleteLater()));
    setUpdatesEnabled(1);
}

void PMainWindow::setCurrentObject(POwlAnimation*obj)
{
    ui->pushButton_kst->hide();
    ui->label_kst->hide();
    ui->pushButtonRemove->show();
    ui->labelRemove->show();
    reconnect(ui->pushButtonRemove,SIGNAL(clicked()),this,SLOT(removeCurrentDataItem()));
    for(int i=0;i<ui->comboBox->count();i++) {
        if(ui->comboBox->itemText(i).contains(obj->idText())) {
            ui->comboBox->setCurrentIndex(i);
            break;
        }
    }
    _currentObject=obj;
}

void PMainWindow::setCurrentObject(PBox*obj)
{
    ui->pushButton_kst->hide();
    ui->label_kst->hide();
    setUpdatesEnabled(0);
    QString x=obj->idText();
    _currentObject=obj;
    bool ok=0;
    for(int i=0;i<ui->comboBox->count();i++) {
        if(ui->comboBox->itemText(i).contains(x)) {
            ok=1;
            ui->comboBox->setCurrentIndex(i);
        }
    }
    Q_ASSERT(ok);
    ui->pstyleCaption->setWidgetStyleRef(obj->_pstyle);
    ui->pushButtonRemove->show();
    ui->labelRemove->show();
    connect(ui->pushButtonRemove,SIGNAL(clicked()),obj,SLOT(deleteLater()));
    setUpdatesEnabled(1);
}

void PMainWindow::setCurrentObject(PAbstractDataItem*obj)
{
    setUpdatesEnabled(0);
    QString x=obj->idText();
    _currentObject=obj;
    for(int i=0;i<ui->comboBox->count();i++) {
        if(ui->comboBox->itemText(i).contains(x)) {
            ui->comboBox->setCurrentIndex(i);
        }
    }

    ui->labelCaption->show();
    ui->lineEditCaption->show();
    reconnect(ui->lineEditCaption,SIGNAL(textChanged(QString)),obj,SLOT(setCaption(QString)));
    reconnect(obj,SIGNAL(textChanged(QString)),this,SLOT(uiLogic()));

    //ui->labelColour->show();

    ui->labelSource->show();
    ui->lineEditSource->show();
    reconnect(ui->lineEditSource,SIGNAL(textChanged(QString)),obj,SLOT(setSource(QString)));
    reconnect(obj,SIGNAL(sourceChanged(QString)),this,SLOT(uiLogic()));

    //ui->label_colour2->show();
    ui->pstyleCaption->show();
    ui->pstyleCaption->setWidgetStyleRef(obj->_captionStyle);

    ui->labelRemove->show();
    ui->pushButtonRemove->show();
    ui->labelRemove->show();
    reconnect(ui->pushButtonRemove,SIGNAL(clicked()),this,SLOT(removeCurrentDataItem()));

    ui->pstyleData->show();
    ui->pstyleData->setWidgetStyleRef(obj->_defaultDataStyle);

    if(dynamic_cast<PNumberDataItem*>(obj)) setCurrentObject(dynamic_cast<PNumberDataItem*>(obj));
    else if(dynamic_cast<PMultiDataItem*>(obj)) setCurrentObject(dynamic_cast<PMultiDataItem*>(obj));
    else if(dynamic_cast<PTimeDataItem*>(obj)) setCurrentObject(dynamic_cast<PTimeDataItem*>(obj));
    else if(dynamic_cast<PDirfileDataItem*>(obj)) {
        ui->labelSource->hide();
        ui->lineEditSource->hide();
    }
    setUpdatesEnabled(1);
}

void PMainWindow::setCurrentObject(PNumberDataItem*obj)
{
    ui->pushButton_kst->show();
    ui->label_kst->show();
    setUpdatesEnabled(0);
    ui->labelFormat_num->show();
    ui->lineEditFormat->show();
    ui->lineEditFormat->setText(obj->format());
    ui->lineEditFormat->setPlaceholderText("printf format str (\"man printf\")");
    reconnect(ui->lineEditFormat,SIGNAL(textChanged(QString)),obj,SLOT(setFormat(QString)));
    reconnect(obj,SIGNAL(formatChanged(QString)),this,SLOT(uiLogic()));

    ui->labelExtrema->show();
    ui->comboBoxExtrema->show();
    reconnect(ui->comboBoxExtrema,SIGNAL(currentIndexChanged(QString)),this,SLOT(extremaLogic(QString)));

    ui->labelName->show();
    ui->lineEditExtremaName->show();
    reconnect(ui->lineEditExtremaName,SIGNAL(textChanged(QString)),this,SLOT(extremaLogic(QString)));
    ui->lineEditExtremaName->setEnabled(obj->_extrema);
    if(obj->_extrema) {
        reconnect(obj->_extrema,SIGNAL(nameChanged(QString)), this,SLOT(extremaLogic(QString)));
    }

    if(obj->_extrema) {
        PExtrema* extrema=obj->_extrema;
        ui->comboBoxExtrema->setCurrentIndex(0);
        for(int i=0;i<ui->comboBoxExtrema->count();i++) {
            if(ui->comboBoxExtrema->itemText(i).contains(extrema->idText())) {
                ui->comboBoxExtrema->setCurrentIndex(i);
                break;
            }
        }
    } else {
        ui->comboBoxExtrema->setCurrentIndex(0);
    }

    ui->labelXHigh->show();
    ui->doubleSpinBoxXHigh->show();
    ui->doubleSpinBoxXHigh->setEnabled(obj->_extrema);
    ui->doubleSpinBoxXHigh->setValue(obj->_extrema?obj->_extrema->_xhigh:0.0f);
    reconnect(ui->doubleSpinBoxXHigh,SIGNAL(valueChanged(double)),this,SLOT(extremaXHighLogic(double)));

    ui->labelHigh->show();
    ui->doubleSpinBoxHigh->show();
    ui->doubleSpinBoxHigh->setEnabled(obj->_extrema);
    ui->doubleSpinBoxHigh->setValue(obj->_extrema?obj->_extrema->_high:0.0f);
    reconnect(ui->doubleSpinBoxHigh,SIGNAL(valueChanged(double)),this,SLOT(extremaHighLogic(double)));

    ui->labelLow->show();
    ui->doubleSpinBoxLow->show();
    ui->doubleSpinBoxLow->setEnabled(obj->_extrema);
    ui->doubleSpinBoxLow->setValue(obj->_extrema?obj->_extrema->_low:0.0f);
    reconnect(ui->doubleSpinBoxLow,SIGNAL(valueChanged(double)),this,SLOT(extremaLowLogic(double)));

    ui->labelXLow->show();
    ui->doubleSpinBoxXLow->show();
    ui->doubleSpinBoxXLow->setEnabled(obj->_extrema);
    ui->doubleSpinBoxXLow->setValue(obj->_extrema?obj->_extrema->_xlow:0.0f);
    reconnect(ui->doubleSpinBoxXLow,SIGNAL(valueChanged(double)),this,SLOT(extremaXLowLogic(double)));

    ui->pstyleXHigh->show();
    ui->pstyleXHigh->setEnabled(obj->_extrema);
    ui->pstyleHigh->show();
    ui->pstyleHigh->setEnabled(obj->_extrema);
    ui->pstyleLow->show();
    ui->pstyleLow->setEnabled(obj->_extrema);
    ui->pstyleXLow->show();
    ui->pstyleXLow->setEnabled(obj->_extrema);
    if(obj->_extrema) {
        ui->pstyleXHigh->setWidgetStyleRef(obj->_extrema->_sxhigh);
        ui->pstyleHigh->setWidgetStyleRef(obj->_extrema->_shigh);
        ui->pstyleLow->setWidgetStyleRef(obj->_extrema->_slow);
        ui->pstyleXLow->setWidgetStyleRef(obj->_extrema->_sxlow);
    }
    setUpdatesEnabled(1);
}

void PMainWindow::setCurrentObject(PMultiDataItem*obj)
{
    ui->pushButton_kst->show();
    ui->label_kst->show();
    setUpdatesEnabled(0);
    disconnect(ui->tableWidgetFormat,0,0,0);

    ui->tableWidgetFormat->clear();
    while(ui->tableWidgetFormat->rowCount()) {
        ui->tableWidgetFormat->removeRow(0);
    }
    for(int i=0;i<obj->_map->_map.size();i++) {
        ui->tableWidgetFormat->insertRow(0);
    }
    QStringList a,b;
    for(int i=0;i<obj->_map->_map.size();i++) {
        a.push_back(QString::number(obj->_map->_map.keys()[i]));
        b.push_back(obj->_map->_map.values()[i]);
    }

    for(int i=0;i<a.size();i++) {
        QTableWidgetItem* twi=new QTableWidgetItem(a[i]);
        ui->tableWidgetFormat->setItem(i,0,twi);
        QTableWidgetItem* twii=new QTableWidgetItem(b[i]);
        ui->tableWidgetFormat->setItem(i,1,twii);
    }

    ui->pstyleSelected->show();
    ui->pstyleSelected->setDisabled(1);

    ui->labelCurRow->show();

    ui->labelFormat_multi->show();
    ui->tableWidgetFormat->show();
    reconnect(ui->tableWidgetFormat,SIGNAL(itemChanged(QTableWidgetItem*)),this,SLOT(multiLogic()));
    reconnect(ui->tableWidgetFormat,SIGNAL(currentCellChanged(int,int,int,int)),this,SLOT(multiLogic()));
    ui->pushButtonAddFormat->show();
    reconnect(ui->pushButtonAddFormat,SIGNAL(clicked()),this,SLOT(multiLogic()));
    ui->pushButtonDelFormat->show();
    reconnect(ui->pushButtonDelFormat,SIGNAL(clicked()),this,SLOT(multiLogic()));
    setUpdatesEnabled(1);
}

void PMainWindow::setCurrentObject(PTimeDataItem*obj)
{
    ui->pushButton_kst->show();
    ui->label_kst->show();
    setUpdatesEnabled(0);
    ui->labelFormat_num->show();
    ui->lineEditFormat->show();
    ui->lineEditFormat->setText(obj->format());
    ui->lineEditFormat->setPlaceholderText("strftime str (\"man strftime\")");
    reconnect(ui->lineEditFormat,SIGNAL(textChanged(QString)),obj,SLOT(setFormat(QString)));
    reconnect(obj,SIGNAL(formatChanged(QString)),this,SLOT(uiLogic()));
    setUpdatesEnabled(1);
}

void PMainWindow::addOwl()
{
    setUpdatesEnabled(0);
    ui->comboBox->addItem("Owl Animation "+_owlList.back()->idText());
    connect(_owlList.back(),SIGNAL(activated()),this,SLOT(uiLogic()));
    _currentObject=_owlList.back();
    ui->comboBox->setCurrentIndex(ui->comboBox->count()-1);

    ui->pushButtonRemove->show();
    ui->labelRemove->show();
    hideEverything();
    setCurrentObject(dynamic_cast<POwlAnimation*>(_owlList.back()));
    setUpdatesEnabled(1);
}

void PMainWindow::uiLogic()
{
    setUpdatesEnabled(0);
    bool newWidget=sender()!=_currentObject||!sender()||dynamic_cast<POwlAnimation*>(sender());

    if(newWidget) {
        hideEverything();

        if(sender()==_mdiArea) addPBox();
        else if(dynamic_cast<PBox*>(sender())) setCurrentObject(dynamic_cast<PBox*>(_currentObject=sender()));
        else if(dynamic_cast<PAbstractDataItem*>(sender())) setCurrentObject(dynamic_cast<PAbstractDataItem*>(_currentObject=sender()));
        else if(dynamic_cast<POwlAnimation*>(sender())) setCurrentObject(dynamic_cast<POwlAnimation*>(_currentObject=sender()));
        else if(sender()==ui->comboBox) {
            QString x=ui->comboBox->currentText();
            x.remove(0,x.indexOf('(')+2);
            x.remove(x.indexOf(')'),999999999);
            Q_ASSERT(PObject::_u[x.toInt()]);
            PObject::_u[x.toInt()]->activate(); // ==> uiLogic();
            setUpdatesEnabled(1);
            return;
        }
    }

    _dirty=1;
    setUpdatesEnabled(1);
}

void PMainWindow::setFileLineEditValidity(QLineEdit* fle)
{
    QPalette pal=fle->palette();
    pal.setColor(fle->foregroundRole(),QFile::exists(fle->text())?"black":"red");
    fle->setPalette(pal);
}

void PMainWindow::curfileLogic(bool force)
{
    setUpdatesEnabled(0);
    setFileLineEditValidity(ui->lineEditHighGain);
    setFileLineEditValidity(ui->lineEditTDRSSOmni);
    setFileLineEditValidity(ui->lineEditIridum);
    setFileLineEditValidity(ui->lineEditLOS);
    QString filename;
    if (ui->radioButtonHighGain->isChecked()) {
        filename = ui->lineEditHighGain->text();
    } else if (ui->radioButtonTDRSSOmni->isChecked()) {
        filename = ui->lineEditTDRSSOmni->text();
    } else if (ui->radioButtonIridum->isChecked()) {
        filename = ui->lineEditIridum->text();
    } else {
        filename = ui->lineEditLOS->text();
    }


    if(QFile::exists(filename) && ((filename != _dirfileFilename) || (force))) {
        delete _dirfile;
        _dirfileFilename=filename;
        _dirfile = new GetData::Dirfile(_dirfileFilename.toAscii(), GD_RDONLY);

        int flc=_dirfile->NFields();
        const char** flv=_dirfile->FieldList();
        QStringList fields;
        for(int i=0;i<flc;i++) {
            fields<<flv[i];
        }
        delete ui->lineEditSource->completer();
        QCompleter* completer=new QCompleter(fields);
        completer->setCaseSensitivity(Qt::CaseInsensitive);
        ui->lineEditSource->setCompleter(completer);
        for(int i=0;i<_pboxList.size();i++) {
            for(int j=0;j<_pboxList[i]->_dataItems.size();j++) {
                _pboxList[i]->_dataItems[j]->resetSource();
            }
        }
    }
    setUpdatesEnabled(1);
}

bool PMainWindow::mouseInactive() {
    return !(ui->actionConfigure->isChecked());
}

void PMainWindow::newLabelLogic(PAbstractDataItem *padi)
{
    setUpdatesEnabled(0);
    ui->comboBox->addItem(padi->caption()+" "+padi->idText());
    connect(padi,SIGNAL(activated()),this,SLOT(uiLogic()));
    padi->mousePressEvent(0);   //make it current
    setUpdatesEnabled(1);
}

void PMainWindow::extremaLogic(QString)
{
    PNumberDataItem* pndi=dynamic_cast<PNumberDataItem*>(_currentObject);
    if(!pndi) return;

    setUpdatesEnabled(0);
    QLineEdit* le=dynamic_cast<QLineEdit*>(sender());
    QComboBox* cb=dynamic_cast<QComboBox*>(sender());
    PExtrema* ex=dynamic_cast<PExtrema*>(sender());
    if(le&&pndi->_extrema) {    //change current extrema name
        Q_ASSERT(pndi->_extrema);
        pndi->_extrema->setName(le->text());
    } else if(cb) {             //change current extrema
        if(pndi->_extrema) {
            disconnect(pndi->_extrema,SIGNAL(nameChanged(QString)), this,SLOT(extremaLogic(QString)));
        }
        PExtrema* extrema=0;
        for(int i=0;i<PExtrema::_u.count();i++) {
            if(cb->currentText().contains(PExtrema::_u[i]->idText())) {
                extrema=PExtrema::_u[i];
            }
        }

        ui->pstyleXHigh->setEnabled(extrema);
        ui->pstyleHigh->setEnabled(extrema);
        ui->pstyleLow->setEnabled(extrema);
        ui->pstyleXLow->setEnabled(extrema);

        if(extrema) {
            pndi->_extrema=extrema;
            ui->lineEditExtremaName->setText(extrema->name());
            ui->lineEditExtremaName->setEnabled(1);
            ui->doubleSpinBoxXHigh->setEnabled(1);
            ui->doubleSpinBoxXHigh->setValue(pndi->_extrema->_xhigh);
            ui->doubleSpinBoxHigh->setEnabled(1);
            ui->doubleSpinBoxHigh->setValue(pndi->_extrema->_high);
            ui->doubleSpinBoxLow->setEnabled(1);
            ui->doubleSpinBoxLow->setValue(pndi->_extrema->_low);
            ui->doubleSpinBoxXLow->setEnabled(1);
            ui->doubleSpinBoxXLow->setValue(pndi->_extrema->_xlow);
            disconnect(pndi->_extrema,SIGNAL(nameChanged(QString)), this,SLOT(extremaLogic(QString)));
            connect(pndi->_extrema,SIGNAL(nameChanged(QString)), this,SLOT(extremaLogic(QString)));

            ui->pstyleXHigh->setWidgetStyleRef(pndi->_extrema->_sxhigh);
            ui->pstyleHigh->setWidgetStyleRef(pndi->_extrema->_shigh);
            ui->pstyleLow->setWidgetStyleRef(pndi->_extrema->_slow);
            ui->pstyleXLow->setWidgetStyleRef(pndi->_extrema->_sxlow);

        } else if (!extrema) {
            if(cb->currentText()==tr("No Extrema")) {
                pndi->_extrema=0;
                ui->lineEditExtremaName->setText("");
                ui->lineEditExtremaName->setDisabled(1);
                ui->doubleSpinBoxXHigh->setDisabled(1);
                ui->doubleSpinBoxHigh->setDisabled(1);
                ui->doubleSpinBoxLow->setDisabled(1);
                ui->doubleSpinBoxXLow->setDisabled(1);
            } else if(cb->currentText()==tr("New Extrema")) {
                QString x;
                while(1) {
                    bool ok;
                    x=QInputDialog::getText(this,"New Extrema","Extrema name:",QLineEdit::Normal,"",&ok);
                    if(!ok) {
                        setUpdatesEnabled(1);
                        return;
                    }
                    if(!x.isEmpty()&&!x.contains("(")&&!x.contains(")")) break;
                    QMessageBox::information(this,"Bad name","Your extrema is poorly named. Try again.");
                }
                PExtrema* ext=new PExtrema;
                ext->setName(x);
                ui->comboBoxExtrema->addItem(x+" "+ext->idText());
                ui->comboBoxExtrema->setCurrentIndex(ui->comboBoxExtrema->count()-1);
                repaint();
            } else {
                qDebug()<<"Warning: unknown extrema:"<<cb->currentText();
            }
        }
    } else if(ex) {             //change current extrema name
        ui->lineEditExtremaName->setText(ex->name());
        ui->comboBoxExtrema->setItemText(ui->comboBoxExtrema->currentIndex(),ex->name()+" "+ex->idText());
    }
    setUpdatesEnabled(1);
}

void PMainWindow::extremaXHighLogic(double x)
{
    PNumberDataItem* pndi=dynamic_cast<PNumberDataItem*>(_currentObject);
    Q_ASSERT(pndi&&pndi->_extrema);
    pndi->_extrema->_xhigh=x;
}

void PMainWindow::extremaHighLogic(double x)
{
    PNumberDataItem* pndi=dynamic_cast<PNumberDataItem*>(_currentObject);
    Q_ASSERT(pndi&&pndi->_extrema);
    pndi->_extrema->_high=x;
}

void PMainWindow::extremaLowLogic(double x)
{
    PNumberDataItem* pndi=dynamic_cast<PNumberDataItem*>(_currentObject);
    Q_ASSERT(pndi&&pndi->_extrema);
    pndi->_extrema->_low=x;
}

void PMainWindow::extremaXLowLogic(double x)
{
    PNumberDataItem* pndi=dynamic_cast<PNumberDataItem*>(_currentObject);
    Q_ASSERT(pndi&&pndi->_extrema);
    pndi->_extrema->_xlow=x;
}

void PMainWindow::multiLogic()
{
    setUpdatesEnabled(0);
    PMultiDataItem* pmdi=dynamic_cast<PMultiDataItem*>(_currentObject);
    Q_ASSERT(pmdi);
    bool update=0;
    if(sender()==ui->pushButtonAddFormat) {
        ui->tableWidgetFormat->insertRow(ui->tableWidgetFormat->rowCount());
    } else if(sender()==ui->pushButtonDelFormat) {
        ui->tableWidgetFormat->removeRow(1);
        update=1;
    } else if(sender()==ui->tableWidgetFormat) {
        update=1;
    }

    if(update) {
        pmdi->_map->reset();
        for(int i=0;i<ui->tableWidgetFormat->rowCount();i++) {
            if(ui->tableWidgetFormat->item(i,0))
            {
                int a=ui->tableWidgetFormat->item(i,0)->data(Qt::EditRole).toInt();
                QString b;
                if(ui->tableWidgetFormat->columnCount()>=2&&
                        ui->tableWidgetFormat->item(i,1) &&
                        !ui->tableWidgetFormat->item(i,1)->data(Qt::EditRole).isNull()) {
                    b=ui->tableWidgetFormat->item(i,1)->data(Qt::EditRole).toString();
                } else {
                    continue;
                }
                pmdi->_map->set(a,b);
            }
        }

        if(ui->tableWidgetFormat->currentRow()!=-1&&ui->tableWidgetFormat->item(ui->tableWidgetFormat->currentRow(),0)) {
            _currowStyle=pmdi->_map->style(ui->tableWidgetFormat->item(
                                               ui->tableWidgetFormat->currentRow(),0)->text().toInt(),PStyle::noStyle);
            ui->pstyleSelected->setWidgetStyleRef(_currowStyle);
            ui->pstyleSelected->setEnabled(1);
        } else {
            ui->pstyleSelected->setEnabled(0);
        }
    }
    setUpdatesEnabled(1);
}

void PMainWindow::currowLogic()
{
    if(!ui->tableWidgetFormat->isVisible()) return;
    setUpdatesEnabled(0);
    if(ui->tableWidgetFormat->selectedItems().size()==1&&
            ui->tableWidgetFormat->item(ui->tableWidgetFormat->selectedItems()[0]->row(),1))
    {
        QString t=ui->tableWidgetFormat->item(ui->tableWidgetFormat->selectedItems()[0]->row(),1)->text();
        if(t.contains("[style=")) {
            if(t.indexOf("]",t.indexOf("[style="))) {
                t.remove(t.indexOf("[style="),t.indexOf("]"));
            } else {
                t.remove(t.indexOf("[style="),999999999);
            }
        }
        t.push_back("[style="+_currowStyle->name()+" "+_currowStyle->idText()+"]");
        ui->tableWidgetFormat->item(ui->tableWidgetFormat->selectedItems()[0]->row(),1)->setText(t);
    }
    setUpdatesEnabled(1);
}

void PMainWindow::gdUpdate()
{
    if(!_dirfile) {
      return;
    }
    setUpdatesEnabled(0);
    int nFrames = _dirfile->NFrames();
    for(int i=0;i<_pboxList.size();i++) {
        _pboxList[i]->gdUpdate(_dirfile,nFrames);
    }
    for(int i=0;i<_owlList.size();i++) {
        _owlList[i]->gdUpdate(_dirfile,nFrames);
    }
    if(_dirty) {
        bool ok=0;
        for(int i=0;i<_pboxList.size();i++) {
            for(int j=0;j<ui->comboBox->count();j++) {
                if(ui->comboBox->itemText(j).contains(_pboxList[i]->idText())) {
                    ui->comboBox->setItemText(j,_pboxList[i]->boxTitle()+" "+_pboxList[i]->idText());
                }
            }

            if(ui->comboBox->currentText().contains(_pboxList[i]->idText())) {
                Q_ASSERT(!ok);
                ok=1;
                ui->lineEditCaption->setText(_pboxList[i]->boxTitle());
                ui->labelCaption->show();
                ui->lineEditCaption->show();
                //ui->labelColour->show();
                ui->pstyleCaption->show();
                reconnect(ui->lineEditCaption,SIGNAL(textChanged(QString)),_pboxList[i],SLOT(setBoxTitle(QString)));
                reconnect(_pboxList[i],SIGNAL(textChanged(QString)),this,SLOT(uiLogic()));
            }

            for(int j=0;j<_pboxList[i]->_dataItems.size();j++) {
                PAbstractDataItem* padi=_pboxList[i]->_dataItems[j];

                if(ui->comboBox->currentText().contains(padi->idText())) {
                    Q_ASSERT(!ok);  // the reason for this is that we should not have the same PADI in two places.
                    ok=1;
                    int l=ui->lineEditCaption->cursorPosition();
                    ui->lineEditCaption->setText(padi->caption());
                    ui->lineEditCaption->setCursorPosition(l);
                    int v=ui->lineEditSource->cursorPosition();
                    ui->lineEditSource->setText(padi->source());
                    ui->lineEditSource->setCursorPosition(v);
                }
            }
        }
        _dirty=0;
    }
    PStyleNotifier::me->notifyChange();
    setUpdatesEnabled(1);
    serverUpdate();
}

void PMainWindow::serverUpdate()
{
    if(ui->checkboxWeb_server_on->isChecked()) {
        if(!_server) {_server=new PServer(ui->lineEditHtmlPath->text()); }
        if(_server->key!=ui->lineEditHtmlPath->text()) {
            delete _server;
            _server=new PServer(ui->lineEditHtmlPath->text());
        }

        QString layoutxmlx,cssx,dataxmlx;
        QTextStream layout(&layoutxmlx);
        layout<<"<OWL-LAYOUT>";
        QTextStream css(&cssx);
        QTextStream data(&dataxmlx);
        data<<"{\"owlData\": { \"Obj\":[\n";

        QMap<int,int> map;

        map.insert(PStyle::noStyle->id(),0);
        css<<"div.S0"<<"{background-color:"<<PStyle::noStyle->bgColour().name()<<";"<<
             "color:"<<PStyle::noStyle->fgColour().name()<<";font-weight:"<<(PStyle::noStyle->isBold()?"bold":"normal")<<";"<<
             "font-style"<<(PStyle::noStyle->isItalic()?"italic":"normal")<<
             "}"<<"/*"<<PStyle::noStyle->name()<<"*/\n";
        css<<"span.S0"<<"{background-color:"<<PStyle::noStyle->bgColour().name()<<";"<<
             "color:"<<PStyle::noStyle->fgColour().name()<<";font-weight:"<<(PStyle::noStyle->isBold()?"bold":"normal")<<";"<<
             "font-style"<<(PStyle::noStyle->isItalic()?"italic":"normal")<<
             "}"<<"/*"<<PStyle::noStyle->name()<<"*/\n\n";

        for(int i=0;i<PStyle::_u.size();i++) {
            map.insert(PStyle::_u[i]->id(),i+1);
            css<<"div.S"<<QString::number(i+1)<<"{background-color:"<<PStyle::_u[i]->bgColour().name()<<";"<<
                 "color:"<<PStyle::_u[i]->fgColour().name()<<";font-weight:"<<(PStyle::_u[i]->isBold()?"bold":"normal")<<";"<<
                 "font-style"<<(PStyle::noStyle->isItalic()?"italic":"normal")<<
                 "}"<<"/*"<<PStyle::_u[i]->name()<<"*/\n";
            css<<"span.S"<<QString::number(i+1)<<"{background-color:"<<PStyle::_u[i]->bgColour().name()<<";"<<
                 "color:"<<PStyle::_u[i]->fgColour().name()<<";font-weight:"<<(PStyle::_u[i]->isBold()?"bold":"normal")<<";"<<
                 "font-style"<<(PStyle::noStyle->isItalic()?"italic":"normal")<<
                 "}"<<"/*"<<PStyle::_u[i]->name()<<"*/\n\n";
        }
        if(cssx!=oldStyle) {
            oldStyle=cssx;
            ++styleVersion;
        }

        for(int i=0;i<_owlList.size();i++) {
            layout<<"<Owl> "
                 "<top>"<<_owlList[i]->geometry().y()<<"</top>"
                 "<left>"<<_owlList[i]->geometry().x()<<"</left>"
                 "<width>"<<_owlList[i]->geometry().width()-10<<"</width>"
                 "<height>"<<_owlList[i]->geometry().height()-10<<"</height></Owl>\n";
        }

        for(int i=0;i<_pboxList.size();i++) {
                layout<<"<PBox>"<<
                     "<top>"<<_pboxList[i]->geometry().y()<<"</top>"<<
                     "<left>"<<_pboxList[i]->geometry().x()<<"</left>"<<
                     "<width>"<<(_pboxList[i]->geometry().width()-10)<<"</width>"<<
                     "<height>"<<_pboxList[i]->geometry().height()-10<<"</height>"<<
                     "<boxStyle>"<<map.value(_pboxList[i]->getStyle()->id())<<"</boxStyle>"<<
                     "<title>"<<_pboxList[i]->boxTitle()<<"</title>";

            for(int j=0;j<_pboxList[i]->_dataItems.size();j++) {
                PAbstractDataItem* padi=_pboxList[i]->_dataItems[j];
                layout<<"<PDataItem>"<<
                     "<capStyle>"<<map.value(padi->captionStyle()->id())<<"</capStyle>"<<
                     "<caption>"<<padi->caption()<<"</caption>"<<
                     "<dataID>"<<padi->id()<<"</dataID></PDataItem>\n";

                if(padi->_serverDirty<0) {
                    data<<"\t{\"i\":"<<padi->id()<<","<<"\"d\":\""<<padi->data()<<"\","<<"\"s\":\""<<
                          map.value(padi->getPrevDataStyle()->id())<<"\"},\n";
                    padi->_serverDirty=5;
                }
            }
            layout<<"</PBox>\n";
        }
        layout<<"</OWL-LAYOUT>";
        if(layoutxmlx!=oldLayout) {
            oldLayout=layoutxmlx;
            ++layoutVersion;
        }
        data<<"{}],";
        data<<"\t\"styleVer\":"<<styleVersion<<",\n";
        data<<"\t\"layoutVer\":"<<layoutVersion<<",\n";
        if(_owlList.size()) {
            data<<"\t\"owlStage\":"<<_owlList[0]->stage()<<"\n";
        } else {
            data<<"\t\"owlStage\":"<<-1<<"\n";
        }
        data<<"} }";

        QFile htmlFile(":/client/PClient.html");
        htmlFile.open(QFile::ReadOnly);
        QString html=htmlFile.readAll();
        html.replace("SERVERNAME",_server->key);

        _server->clockOn(html,cssx,layoutxmlx,dataxmlx);
    }
}

void PMainWindow::removeCurrentDataItem()
{
    PAbstractDataItem* padi=dynamic_cast<PAbstractDataItem*>(_currentObject);
    POwlAnimation* owl=dynamic_cast<POwlAnimation*>(_currentObject);
    if(!padi&&!owl) return;
    else if(owl) {
        for(int i=0;i<_owlList.size();i++) {
            if(_owlList[i]==owl) {
                obviate(owl);
                ui->comboBox->setCurrentIndex(0);
                owl->deleteLater();
                hideEverything();
                activate();
            }
        }
        return;
    }
    for(int i=0;i<_pboxList.size();i++) {
        _pboxList[i]->_dataItems.removeOne(padi);
    }
    obviate(padi);
    ui->comboBox->setCurrentIndex(0);
    _currentObject->deleteLater();
    hideEverything();
    ui->comboBox->setCurrentIndex(0);
    activate();
}

void PMainWindow::obviate(POwlAnimation *byeBye)
{
    for(int i=0;i<ui->comboBox->count();i++) {
        if(ui->comboBox->itemText(i).contains(byeBye->idText())) {
            ui->comboBox->removeItem(i);
            break;
        }
    }
    activate();
}

void PMainWindow::obviate(PBox *byeBye)
{
    ui->comboBox->setCurrentIndex(0);
    ui->pstyleCaption->setNoWidgetStyleRef();
    ui->pstyleData->setNoWidgetStyleRef();
    _pboxList.removeOne(byeBye);
    for(int i=0;i<ui->comboBox->count();i++) {
        if(ui->comboBox->itemText(i).contains(byeBye->idText())) {
            ui->comboBox->removeItem(i);
            break;
        }
    }
    for(int j=0;j<byeBye->_dataItems.size();j++) {
        obviate(byeBye->_dataItems[j]);
    }
    hideEverything();
    ui->comboBox->setCurrentIndex(0);
    activate();
}

void PMainWindow::obviate(PAbstractDataItem *byeBye)
{
    ui->comboBox->setCurrentIndex(0);
    ui->pstyleCaption->setNoWidgetStyleRef();
    ui->pstyleData->setNoWidgetStyleRef();
    ui->pstyleHigh->setNoWidgetStyleRef();
    ui->pstyleLow->setNoWidgetStyleRef();
    ui->pstyleSelected->setNoWidgetStyleRef();
    ui->pstyleXHigh->setNoWidgetStyleRef();
    ui->pstyleXLow->setNoWidgetStyleRef();

    for(int i=0;i<ui->comboBox->count();i++) {
        if(ui->comboBox->itemText(i).contains(byeBye->idText())) {
            ui->comboBox->removeItem(i);
            break;
        }
    }
    activate();
}

void PMainWindow::recognizeExtrema(PExtrema *e)
{
    ui->comboBoxExtrema->addItem(e->name()+" "+e->idText());
}

void PMainWindow::activate()
{
    reconnect(ui->pushButtonResetLink, SIGNAL(clicked()), this, SLOT(resetLink()));
    reconnect(ui->lineEditLOS,SIGNAL(textChanged(QString)),this,SLOT(curfileLogic()));
    reconnect(ui->lineEditHighGain,SIGNAL(textChanged(QString)),this,SLOT(curfileLogic()));
    reconnect(ui->lineEditIridum,SIGNAL(textChanged(QString)),this,SLOT(curfileLogic()));
    reconnect(ui->lineEditTDRSSOmni,SIGNAL(textChanged(QString)),this,SLOT(curfileLogic()));
    reconnect(ui->radioButtonLOS,SIGNAL(clicked()),this,SLOT(curfileLogic()));
    reconnect(ui->radioButtonHighGain,SIGNAL(clicked()),this,SLOT(curfileLogic()));
    reconnect(ui->radioButtonIridum,SIGNAL(clicked()),this,SLOT(curfileLogic()));
    reconnect(ui->radioButtonTDRSSOmni,SIGNAL(clicked()),this,SLOT(curfileLogic()));
}

QVariant save(PMainWindow&b);
void PMainWindow::owlSave()
{
    QString filename = _filename; //settings->value("filename").toString();
    QFileInfo qfi(filename);
    if (qfi.exists()) {
        QFile file(filename);
        file.open(QFile::WriteOnly | QFile::Text);
        QJson::Serializer s;
        file.write(s.serialize(save(*this)).replace('{',"\n{").replace('}',"}\n"));
        _settings->setValue("filename", file.fileName());

        file.close();
    } else {
        owlSaveAs();
    }
}

void PMainWindow::owlSaveAs()
{
    QString filename = _filename; //settings->value("filename").toString();
    QFileInfo qfi(filename);

    QFile file(QFileDialog::getSaveFileName(this,"Save the current owl project",qfi.dir().dirName(),"Owl projects(*.owl)"));
    file.open(QFile::WriteOnly | QFile::Text);
    QJson::Serializer s;
    file.write(s.serialize(save(*this)).replace('{',"\n{").replace('}',"}\n"));
    _settings->setValue("filename", file.fileName());
    _filename = file.fileName();

    file.close();
}


void load(QVariant v,PMainWindow&b);
void PMainWindow::owlLoad(QString filename)
{
    if(filename.isEmpty()) {
        filename = QFileDialog::getOpenFileName(0,"Load a pal/owl project","","Pal/Owl projects(*.pal *.owl)");
    }

    QFileInfo f(filename);

    if (!f.exists()) {
        return;
    }

    filename=f.absoluteFilePath();

    if(filename.isEmpty()) {
        return;
    }
    setUpdatesEnabled(0);

    ///////////////////////////////////////////
    _deleteScheduled=1;
    deleteLater();
    PExtrema::_u.clear();
    PStyle::_u.clear();
    PObject::_u.clear();
    PStyleNotifier::me->disable();
    ///////////////////////////////////////////

    if(filename.endsWith("owl")) {
        QFile file(filename);
        file.open(QFile::ReadOnly);
        QJson::Parser p;
        bool ok;
        QVariant v=p.parse(file.readAll(),&ok);
        if(ok&&v.isValid()) {
            PObject::isLoading=1;
            PMainWindow* evenNewer=new PMainWindow(0);
            evenNewer->setUpdatesEnabled(0);
            evenNewer->_ut->stop();
            load(v,*evenNewer);
            evenNewer->setWindowTitle(_WINDOW_TITLE_);
            evenNewer->show();
            evenNewer->_ut->start();
            evenNewer->setUpdatesEnabled(1);
            file.close();
        } else {    //try legacy format
            PObject::isLoading=1;
            qDebug()<<"Trying to open legacy...";
            file.close();
            QFile file(filename);
            file.open(QFile::ReadOnly);
            QDataStream xds(&file);
            PMainWindow* evenNewer=new PMainWindow(0);
            PObject::isLoading=0;
            evenNewer->setUpdatesEnabled(0);
            evenNewer->_ut->stop();
            xds>>*evenNewer;
            evenNewer->setWindowTitle(_WINDOW_TITLE_);
            evenNewer->show();
            evenNewer->_ut->start();
            evenNewer->setUpdatesEnabled(1);
            file.close();
        }
    } else if(filename.endsWith("pal")) {
        PObject::isLoading=1;
        PMainWindow* newMain=new PMainWindow(0);
        PObject::isLoading=0;
        qApp->setActiveWindow(newMain);
        PDotPal dotPal(filename);
        for(int i=0;i<dotPal._pbox.size();i++) {
            newMain->_mdiArea->createPBox(0,0,dotPal._pbox[i]);
            newMain->_pboxList.back()->show();
        }
        newMain->show();
    } else {
        QMessageBox::warning(0,"Could not load file",filename+" does not seem to be either a .owl file or a .pal file!");

        PMainWindow* newMain=new PMainWindow(0);
        qApp->setActiveWindow(newMain);
        newMain->show();
        PStyleNotifier::me->enable();
        PStyleNotifier::me->notifyChange();
        return;
    }

    _settings->setValue("filename", filename);
    _filename = filename;
    PStyleNotifier::me->enable();
    PStyleNotifier::me->notifyChange();
    setUpdatesEnabled(1);
}

void PMainWindow::showInKst() {
    PAbstractDataItem* padi=dynamic_cast<PAbstractDataItem*>(currentObject());
    if(!padi) return;
    PyRun_SimpleString(QString("import pykst as kst\n"
                       "client = kst.Client(\""+QString(getenv("USER"))+"-owl\")\n"    //this should be USERNAME for windows
                       "x=kst.DataVector(client,\""+QString(_dirfileFilename)+"\",\"INDEX\")\n"
                       "y=kst.DataVector(client,\""+QString(_dirfileFilename)+"\",\""+QString(padi->source())+"\")\n"
                       "client.plot(x,y)\n").toAscii());
}
