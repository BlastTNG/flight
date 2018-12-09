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
#include "PStyle.h"
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
#include <QDateTime>
#include <QMenu>
#include <QWidgetAction>

#ifdef __APPLE__
#include <python2.6/Python.h>
#else
#include <python2.7/Python.h>   //you may need to change this
#endif


#if QT_VERSION >= 0x050000
#include <QJsonDocument>
#include <QJsonObject>
#else
#include <qjson/parser.h>
#include <qjson/serializer.h>
#endif

PMainWindow* PMainWindow::me=0;

static QString _filename;

PMainWindow::PMainWindow(int font_size, QString file, QWidget *parent) :
    QMainWindow(parent),
    PObject(0),
    _currentObject(0),
    _dirfile(0),
    _scrollArea(new QScrollArea()),
    styleVersion(0),
    layoutVersion(0),
#if QT_VERSION >= 0x050300
    _server(0),
#endif
    _deleteScheduled(0),
    _lastNFrames(-1),
    ui(new Ui::PMainWindow)
{
    _link = 0; /* LOS */
    _settings = new QSettings("UToronto", "owl");
    setWindowIcon(QIcon(":icons/Owl0.png"));

    PStyle::noStyle = PStyle::noStyle?PStyle::noStyle:new PStyle("No style");
    me=this;

    QFont my_font = font();
    my_font.setPointSize(font_size);
    setFont(my_font);

    ui->setupUi(this);

    int listHeight = ui->listWidgetInsertReal->fontMetrics().height() *
                     ui->listWidgetInsertReal->count() * 1.05 + 2*ui->listWidgetInsertReal->frameWidth();

    ui->listWidgetInsertReal->setMaximumHeight(listHeight);
    ui->comboBox->addItem("Owl "+idText());
    _mdiArea=new PMdiArea;
    _scrollArea->setParent(centralWidget());
    _scrollArea->setAutoFillBackground(1);
    QPalette p=_scrollArea->palette();
    p.setColor(_scrollArea->backgroundRole(),"white");
    _scrollArea->setPalette(p);
    _scrollArea->setWidget(_mdiArea);
    _mdiArea->adjustSize();
    centralWidget()->layout()->addWidget(_scrollArea);
    connect(_mdiArea,SIGNAL(newBox(PBox*)),this,SLOT(uiLogic()));
    connect(ui->comboBox,SIGNAL(activated(int)),this,SLOT(uiLogic()));
    connect(ui->actionSave,SIGNAL(triggered()),this,SLOT(owlSave()));
    connect(ui->actionSaveAs,SIGNAL(triggered()),this,SLOT(owlSaveAs()));
    connect(ui->actionLoad,SIGNAL(triggered()),this,SLOT(owlLoad()));
    connect(ui->actionReset, SIGNAL(triggered()), this, SLOT(resetLink()));
    connect(PStyleNotifier::me,SIGNAL(change()),this,SLOT(currowLogic()));
    connect(ui->actionHelp,SIGNAL(triggered()),this,SLOT(readmeHelp()));
    connect(_mdiArea,SIGNAL(newOwl(POwlAnimation*)),this,SLOT(addOwl()));
#if QT_VERSION >= 0x050300
    connect(ui->checkBoxServer, SIGNAL(toggled(bool)), this, SLOT(setWebEnabled(bool)));
    connect(ui->spinBoxServerPort, SIGNAL(valueChanged(int)), this, SLOT(setWebPort(int)));
#else
    ui->actionServer_Options->setVisible(false);
#endif
    QFileSystemModel* fsm=new QFileSystemModel();
    fsm->setRootPath(QDir::currentPath());
    ui->lineEditHighGain->setCompleter(new QCompleter(fsm));
    ui->lineEditTDRSSOmni->setCompleter(new QCompleter(fsm));
    ui->lineEditLOS->setCompleter(new QCompleter(fsm));
    ui->lineEditIridium->setCompleter(new QCompleter(fsm));
    ui->lineEditLoRate->setCompleter(new QCompleter(fsm));

    curfileLogic();

    _currowStyle=PStyle::noStyle;

    uiLogic();

    _ut=new QTimer(this);
    _ut->setInterval(500);
    connect(_ut,SIGNAL(timeout()),this,SLOT(gdUpdate()));
    _ut->start();

    _reset_timer=new QTimer(this);
    _reset_timer->setInterval(30000); /* twice a minute is good...? */
    connect(_reset_timer,SIGNAL(timeout()),this,SLOT(resetLink()));
    _reset_timer->start();

    activate();

    //_mdiArea->setMinimumSize(1000,900);
    setMinimumSize(1,1);

    //setWindowTitle(_WINDOW_TITLE_);

    if (file == "__lastfile") {
        file = _settings->value("filename").toString();
    }

    restoreState(_settings->value("windowState").toByteArray());

    //if (_settings->value("hideWeb", false).toBool()) {
        ui->actionServer_Options->setChecked(false);
        ui->dockWebServer->hide();
    //}
    //if (_settings->value("hideConfig", false).toBool()) {
      ui->actionConfigure->setChecked(false);
      ui->dockConfigure->hide();
    //}
    //if (_settings->value("hideLink", false).toBool()) {
      ui->actionLink->setChecked(false);
      ui->dockLink->hide();
    //}
    if(file.size()) {
        owlLoad(file);
        _filename = file;
    }

    setStatusBar(0);

    if(!_deleteScheduled&&!PObject::isLoading) show();
}

#define reconnect(a,b,c,d) \
    disconnect(a,b,c,d); \
    connect(a,b,c,d)

PMainWindow::~PMainWindow()
{
    while(_pboxList.size()) {
        delete _pboxList.takeFirst();
    }
    while(_owlList.size()) {
        delete _owlList.takeFirst();
    }
}

void PMainWindow::keyPressEvent ( QKeyEvent * e ) {
  if (e->key()== Qt::Key_T) {
    ui->toolBar->setVisible(!ui->toolBar->isVisible());
  }
  QMainWindow::keyPressEvent(e);
}

void PMainWindow::contextMenuEvent(QContextMenuEvent *event) {

  QMenu menu;

#if 0
  QWidgetAction *title = new QWidgetAction(&menu);
  title->setEnabled(false);

  QLabel *label = new QLabel("Test Title");
  label->setAlignment(Qt::AlignCenter);
  label->setStyleSheet("QLabel {"
                       "border-bottom: 2px solid lightGray;"
                       "font: bold large;"
                       "padding: 3px;"
                       "margin: 1px;"
                       "}");
  title->setDefaultWidget(label);
  menu.addAction(title);
#endif

  QAction *show_toolbar_action = new QAction(tr("Show Toolbar (T)"),this);
  show_toolbar_action->setCheckable(true);
  show_toolbar_action->setChecked(!ui->toolBar->isHidden());
  connect(show_toolbar_action, SIGNAL(toggled(bool)), this, SLOT(showToolbar(bool)));

  menu.addAction(show_toolbar_action);

  menu.exec(event->globalPos());
  //QMainWindow::contextMenuEvent(event);
}

void PMainWindow::showToolbar(bool show) {
  if (show) {
    ui->toolBar->show();
  } else {
    ui->toolBar->hide();
  }
}

void PMainWindow::closeEvent(QCloseEvent* e)
{
  Q_UNUSED(e);

/*
  _settings->setValue("hideWeb",!ui->dockWebServer->isVisible());
  _settings->setValue("hideConfig",!ui->dockConfigure->isVisible());
  _settings->setValue("hideLink",!ui->dockLink->isVisible());
*/
  _settings->setValue("windowState", saveState());
  //e->setAccepted(QMessageBox::question(this,"Really Quit?","If you quit, you will lose all unsaved data. Quit anyway?",QMessageBox::Yes,QMessageBox::No)==QMessageBox::Yes);
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
    ui->labelNBits->hide();
    ui->spinBoxNBits->hide();
    disconnect(ui->spinBoxNBits,0,0,0);
    ui->labelHighWord->hide();
    ui->lineEditHighWord->hide();
    disconnect(ui->lineEditHighWord,0,0,0);
    ui->labelLowWord->hide();
    ui->lineEditLowWord->hide();
    disconnect(ui->lineEditLowWord,0,0,0);
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
    connect(ui->pushButtonRemove,SIGNAL(clicked()),_pboxList.back(),SLOT(deleteLater()));
    setUpdatesEnabled(1);
}

void PMainWindow::setCurrentObject(POwlAnimation*obj)
{
    ui->pushButtonRemove->show();
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

    ui->pushButtonRemove->show();
    reconnect(ui->pushButtonRemove,SIGNAL(clicked()),this,SLOT(removeCurrentDataItem()));

    ui->pstyleData->show();
    ui->pstyleData->setWidgetStyleRef(obj->_defaultDataStyle);

    if(dynamic_cast<PExtremaDataItem*>(obj))
      setCurrentObject(dynamic_cast<PExtremaDataItem*>(obj));

    if(dynamic_cast<PNumberDataItem*>(obj)) setCurrentObject(dynamic_cast<PNumberDataItem*>(obj));
    else if(dynamic_cast<PMultiDataItem*>(obj)) setCurrentObject(dynamic_cast<PMultiDataItem*>(obj));
    else if(dynamic_cast<PBitMultiDataItem*>(obj)) setCurrentObject(dynamic_cast<PBitMultiDataItem*>(obj));
    else if(dynamic_cast<PTimeDataItem*>(obj)) setCurrentObject(dynamic_cast<PTimeDataItem*>(obj));
    else if(dynamic_cast<PDirfileDataItem*>(obj)) {
        ui->labelSource->hide();
        ui->lineEditSource->hide();
    }
    setUpdatesEnabled(1);
}

void PMainWindow::setCurrentObject(PNumberDataItem*obj)
{
    setUpdatesEnabled(0);
    ui->labelFormat_num->show();
    ui->lineEditFormat->show();
    ui->lineEditFormat->setText(obj->format());
    ui->lineEditFormat->setPlaceholderText("printf format str (\"man printf\")");
    reconnect(ui->lineEditFormat,SIGNAL(textChanged(QString)),obj,SLOT(setFormat(QString)));
    reconnect(obj,SIGNAL(formatChanged(QString)),this,SLOT(uiLogic()));
    setUpdatesEnabled(1);
}

void PMainWindow::setCurrentObject(PExtremaDataItem *obj)
{
    setUpdatesEnabled(0);
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

void PMainWindow::setCurrentObject(PBitMultiDataItem*obj)
{
    setUpdatesEnabled(0);
    ui->labelNBits->show();
    ui->spinBoxNBits->show();
    ui->spinBoxNBits->setValue(obj->nBits());
    reconnect(ui->spinBoxNBits,SIGNAL(valueChanged(int)),obj,SLOT(setNBits(int)));
    reconnect(obj,SIGNAL(nBitsChanged(int)),this,SLOT(uiLogic()));

    ui->labelHighWord->show();
    ui->lineEditHighWord->show();
    ui->lineEditHighWord->setText(obj->highWord());
    ui->lineEditHighWord->setPlaceholderText("11111111");
    reconnect(ui->lineEditHighWord,SIGNAL(textChanged(QString)),obj,SLOT(setHighWord(QString)));
    reconnect(obj,SIGNAL(highWordChanged(QString)),this,SLOT(uiLogic()));

    ui->labelLowWord->show();
    ui->lineEditLowWord->show();
    ui->lineEditLowWord->setText(obj->lowWord());
    ui->lineEditLowWord->setPlaceholderText("00000000");
    reconnect(ui->lineEditLowWord,SIGNAL(textChanged(QString)),obj,SLOT(setLowWord(QString)));
    reconnect(obj,SIGNAL(lowWordChanged(QString)),this,SLOT(uiLogic()));

    setUpdatesEnabled(1);
}

void PMainWindow::setCurrentObject(PTimeDataItem*obj)
{
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

/* Set the current link based on an .owl file value */
void PMainWindow::setLink(int link, QStringList linkNames)
{
  /* LOS is the default link */
  if (link < 0 || link > 4)
    link = 0;

  if (linkNames.size()>4) {
    ui->lineEditLOS->setText(linkNames[0]);
    ui->lineEditIridium->setText(linkNames[1]);
    ui->lineEditTDRSSOmni->setText(linkNames[2]);
    ui->lineEditHighGain->setText(linkNames[3]);
    ui->lineEditLoRate->setText(linkNames[4]);
  }
  ui->radioButtonLoRate->setChecked(link == 4);
  ui->radioButtonHighGain->setChecked(link == 3);
  ui->radioButtonTDRSSOmni->setChecked(link == 2);
  ui->radioButtonIridium->setChecked(link == 1);
  ui->radioButtonLOS->setChecked(link == 0);

  /* do the link logic */
  curfileLogic(true);
}

void PMainWindow::curfileLogic(bool force)
{
    setUpdatesEnabled(0);
    setFileLineEditValidity(ui->lineEditLoRate);
    setFileLineEditValidity(ui->lineEditHighGain);
    setFileLineEditValidity(ui->lineEditTDRSSOmni);
    setFileLineEditValidity(ui->lineEditIridium);
    setFileLineEditValidity(ui->lineEditLOS);
    QString filename;
    if (ui->radioButtonLoRate->isChecked()) {
        filename = ui->lineEditLoRate->text();
        _link = 4;
    } else if (ui->radioButtonHighGain->isChecked()) {
        filename = ui->lineEditHighGain->text();
        _link = 3;
    } else if (ui->radioButtonTDRSSOmni->isChecked()) {
        filename = ui->lineEditTDRSSOmni->text();
        _link = 2;
    } else if (ui->radioButtonIridium->isChecked()) {
        filename = ui->lineEditIridium->text();
        _link = 1;
    } else {
        filename = ui->lineEditLOS->text();
        _link = 0;
    }

    if(QFile::exists(filename) && ((filename != _dirfileFilename) || (force))) {
        delete _dirfile;
        _dirfileFilename=filename;
        _dirfile = new GetData::Dirfile(_dirfileFilename.toLatin1(), GD_RDONLY);

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

    setWindowTitle("Owl - " + filename);
}

bool PMainWindow::mouseInactive() {
    return !(ui->actionConfigure->isChecked());
}

QStringList PMainWindow::linkNames() {
  QStringList names;

  names.append(ui->lineEditLOS->text());
  names.append(ui->lineEditIridium->text());
  names.append(ui->lineEditTDRSSOmni->text());
  names.append(ui->lineEditHighGain->text());
  names.append(ui->lineEditLoRate->text());

  return names;
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
    PExtremaDataItem* pedi=dynamic_cast<PExtremaDataItem*>(_currentObject);
    if (!pedi)
      return;

    setUpdatesEnabled(0);
    QLineEdit* le=dynamic_cast<QLineEdit*>(sender());
    QComboBox* cb=dynamic_cast<QComboBox*>(sender());
    PExtrema* ex=dynamic_cast<PExtrema*>(sender());
    if (le && pedi->_extrema) {    //change current extrema name
        Q_ASSERT(pedi->_extrema);
        pedi->_extrema->setName(le->text());
    } else if(cb) {             //change current extrema
        if (pedi->_extrema) {
            disconnect(pedi->_extrema, SIGNAL(nameChanged(QString)), this,
                    SLOT(extremaLogic(QString)));
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
            pedi->_extrema = extrema;
            ui->lineEditExtremaName->setText(extrema->name());
            ui->lineEditExtremaName->setEnabled(1);
            ui->doubleSpinBoxXHigh->setEnabled(1);
            ui->doubleSpinBoxXHigh->setValue(pedi->_extrema->_xhigh);
            ui->doubleSpinBoxHigh->setEnabled(1);
            ui->doubleSpinBoxHigh->setValue(pedi->_extrema->_high);
            ui->doubleSpinBoxLow->setEnabled(1);
            ui->doubleSpinBoxLow->setValue(pedi->_extrema->_low);
            ui->doubleSpinBoxXLow->setEnabled(1);
            ui->doubleSpinBoxXLow->setValue(pedi->_extrema->_xlow);
            disconnect(pedi->_extrema, SIGNAL(nameChanged(QString)), this,
                    SLOT(extremaLogic(QString)));
            connect(pedi->_extrema, SIGNAL(nameChanged(QString)), this,
                    SLOT(extremaLogic(QString)));

            ui->pstyleXHigh->setWidgetStyleRef(pedi->_extrema->_sxhigh);
            ui->pstyleHigh->setWidgetStyleRef(pedi->_extrema->_shigh);
            ui->pstyleLow->setWidgetStyleRef(pedi->_extrema->_slow);
            ui->pstyleXLow->setWidgetStyleRef(pedi->_extrema->_sxlow);

        } else if (!extrema) {
            if(cb->currentText()==tr("No Extrema")) {
                pedi->_extrema = 0;
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
    PExtremaDataItem* pedi=dynamic_cast<PExtremaDataItem*>(_currentObject);
    Q_ASSERT(pedi && pedi->_extrema);
    pedi->_extrema->_xhigh=x;
}

void PMainWindow::extremaHighLogic(double x)
{
    PExtremaDataItem* pedi=dynamic_cast<PExtremaDataItem*>(_currentObject);
    Q_ASSERT(pedi && pedi->_extrema);
    pedi->_extrema->_high=x;
}

void PMainWindow::extremaLowLogic(double x)
{
    PExtremaDataItem* pedi=dynamic_cast<PExtremaDataItem*>(_currentObject);
    Q_ASSERT(pedi&&pedi->_extrema);
    pedi->_extrema->_low=x;
}

void PMainWindow::extremaXLowLogic(double x)
{
    PExtremaDataItem* pedi=dynamic_cast<PExtremaDataItem*>(_currentObject);
    Q_ASSERT(pedi && pedi->_extrema);
    pedi->_extrema->_xlow=x;
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

void PMainWindow::bitmultiLogic()
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
    if (!_dirfile) {
      return;
    }

    static int last_nFrames = 0;
    int nFrames = _dirfile->NFrames();

    if (nFrames != last_nFrames) {
        last_nFrames = nFrames;
        /* reset the resetLink timer */
        _reset_timer->start();
    }
    PAbstractDataItem::newCycle(); // reset delay timeout so
    // item updates don't take too long
    for(int i=0;i<_pboxList.size();i++) {
        _pboxList[i]->gdUpdate(_dirfile,nFrames);
    }


    if (_lastNFrames != nFrames) {
      _lastNFrames = nFrames;
      _lastUpdate = time(NULL);
    }

    int dT = time(NULL) - _lastUpdate;
    for(int i=0;i<_owlList.size();i++) {
        _owlList[i]->gdUpdate(_dirfile,nFrames, dT);
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
    serverUpdate();
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

QVariant PMainWindow::state()
{
    QVariantMap state;
    state.insert("statics", save(*this));
    state.insert("data", _data());
    return state;
}

QVariant PMainWindow::_data()
{
    QVariantMap data;
    for (int i = 0; i < _pboxList.size(); ++i) {
        for (int j = 0; j < _pboxList[i]->_dataItems.size(); ++j) {
            PAbstractDataItem* padi = _pboxList[i]->_dataItems[j];
            QVariantList list;
            list.push_back(padi->data());
            list.push_back(padi->getPrevDataStyle()->id());
            data.insert("(P" + QString::number(padi->id()) + ")", list);
        }
    }
    return data;
}

QVariant PMainWindow::stateChanges()
{
    // This does not currently send changes to layout and such.
    QVariantMap state;

    QVariantMap data;
    for (int i = 0; i < _pboxList.size(); ++i) {
        for (int j = 0; j < _pboxList[i]->_dataItems.size(); ++j) {
            PAbstractDataItem* padi = _pboxList[i]->_dataItems[j];
            if (padi->_serverDirty) {
              QVariantList list;
              list.push_back(padi->data());
              list.push_back(padi->getPrevDataStyle()->id());
              data.insert("(P" + QString::number(padi->id()) + ")", list);
              //data.insert("(P" + QString::number(padi->id()) + ")", padi->data());
              padi->_serverDirty = false;
            }
        }
    }

    state.insert("data", data);
    return state;
}

void PMainWindow::serverUpdate()
{
#if QT_VERSION >= 0x050300
    if (_server) {
        _server->update();
    }
#endif
}

#if QT_VERSION >= 0x050300
int PMainWindow::webPort()
{
    if (ui->checkBoxServer->isChecked()) {
        return ui->spinBoxServerPort->value();
    } else {
        return -1;
    }
}

void PMainWindow::setWebEnabled(const bool &enabled) {
    this->setWebPort(enabled ? ui->spinBoxServerPort->value() : -1);
}

void PMainWindow::setWebPort(const int &port)
{
    static bool noRecurse = false;
    if (noRecurse) {
        return;
    }
    noRecurse = true;

    delete _server;
    _server = 0;
    if (port == -1) {
        ui->checkBoxServer->setChecked(false);
        ui->spinBoxServerPort->setValue(8001);
    } else {
        ui->checkBoxServer->setChecked(true);
        ui->spinBoxServerPort->setValue(port);
        _server = new PWebServer(this, port);
    }

    noRecurse = false;
}
#endif

void PMainWindow::activate()
{
    reconnect(ui->pushButtonResetLink, SIGNAL(clicked()), this, SLOT(resetLink()));
    reconnect(ui->lineEditLOS,SIGNAL(textChanged(QString)),this,SLOT(curfileLogic()));
    reconnect(ui->lineEditLoRate,SIGNAL(textChanged(QString)),this,SLOT(curfileLogic()));
    reconnect(ui->lineEditHighGain,SIGNAL(textChanged(QString)),this,SLOT(curfileLogic()));
    reconnect(ui->lineEditIridium,SIGNAL(textChanged(QString)),this,SLOT(curfileLogic()));
    reconnect(ui->lineEditTDRSSOmni,SIGNAL(textChanged(QString)),this,SLOT(curfileLogic()));
    reconnect(ui->radioButtonLOS,SIGNAL(clicked()),this,SLOT(curfileLogic()));
    reconnect(ui->radioButtonLoRate,SIGNAL(clicked()),this,SLOT(curfileLogic()));
    reconnect(ui->radioButtonHighGain,SIGNAL(clicked()),this,SLOT(curfileLogic()));
    reconnect(ui->radioButtonIridium,SIGNAL(clicked()),this,SLOT(curfileLogic()));
    reconnect(ui->radioButtonTDRSSOmni,SIGNAL(clicked()),this,SLOT(curfileLogic()));
}

QByteArray formatStream(QByteArray &rawS) {
  QByteArray formatedS;
  int n_spaces = 0;

  formatedS.reserve(rawS.size()*1.5);

  rawS.replace("{ ","{\n").replace('}',"\n}").replace(", \"", ",\n\"");

  for (int i=0; i<rawS.size(); i++) {
    formatedS.append(rawS.at(i));
    if ((i+1<rawS.size()) && (rawS.at(i+1)=='}')) {
      n_spaces--;
    }
    if (rawS.at(i)=='\n') {
      for (int j=0; j<n_spaces*2; j++) {
        formatedS.append(' ');
      }
    } else if (rawS.at(i)=='{') {
      n_spaces++;
    }
  }

  return formatedS;
}

QVariant save(PMainWindow&b);
void PMainWindow::owlSave()
{
    QString filename = _filename; //settings->value("filename").toString();
    QFileInfo qfi(filename);
    if (qfi.exists()) {
        QFile file(filename);
        file.open(QFile::WriteOnly | QFile::Text);

#if QT_VERSION >= 0x050000
        QJsonDocument document = QJsonDocument::fromVariant(save(*this));
        file.write(document.toJson(QJsonDocument::Indented));
#else
        QJson::Serializer s;
        QByteArray rawS(s.serialize(save(*this)));
        file.write(formatStream(rawS));
        //file.write(rawS);
#endif

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

#if QT_VERSION >= 0x050000
    QJsonDocument document = QJsonDocument::fromVariant(save(*this));
    file.write(document.toJson(QJsonDocument::Indented));
#else
    QJson::Serializer s;
    QByteArray rawS(s.serialize(save(*this)));
    file.write(formatStream(rawS));
    //file.write(rawS);
#endif

    _settings->setValue("filename", file.fileName());

    _filename = file.fileName();

    file.close();
}


void load(QVariant v,PMainWindow&b);
void PMainWindow::owlLoad(QString filename)
{
    if(filename.isEmpty()) {
        filename = QFileDialog::getOpenFileName(0,"Load an Owl project","","Owl projects(*.owl)");
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
        QByteArray qba = file.readAll();

#if QT_VERSION >= 0x050000
        QJsonParseError error;

        QJsonDocument d = QJsonDocument::fromJson(qba, &error);
        QVariant root = d.toVariant();

        if (error.error != QJsonParseError::NoError) {
            QMessageBox::critical(0, "Parse Error", "Could not parse JSON: " + error.errorString());
            return;
        }
#else
        QJson::Parser p;  // QT4
        bool ok;
        QVariant root=p.parse(qba,&ok); // QT4

        if (!(ok&&root.isValid())) {
          QMessageBox::critical(0, "Parse Error", "Could not parse JSON: ");
          return;
        }
#endif

        PObject::isLoading=1;
        PMainWindow* newWindow=new PMainWindow(font().pointSize(), 0);
        newWindow->setUpdatesEnabled(0);
        newWindow->_ut->stop();
        newWindow->_reset_timer->stop();
        load(root,*newWindow);

        newWindow->_mdiArea->adjustSize();
        newWindow->_mdiArea->setMinimumSize(newWindow->width()-70, newWindow->height()-70);

        //newWindow->setWindowTitle(_WINDOW_TITLE_);
        newWindow->show();
        newWindow->_ut->start();
        newWindow->_reset_timer->start();
        newWindow->setUpdatesEnabled(1);
        file.close();
    } else {
        QMessageBox::warning(0,"Could not load file",filename+" does not seem to be either a .owl file!");

        PMainWindow* newMain=new PMainWindow(font().pointSize(), 0);
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

void PMainWindow::setMDIMinSize(int w, int h) {
  _mdiArea->setMinimumSize(QSize(w, h));
}

void PMainWindow::showInKst() {
    PAbstractDataItem* padi=dynamic_cast<PAbstractDataItem*>(currentObject());
    if(!padi) return;
    /*
    qDebug() << QString("import pykst as kst\n"
                        "client = kst.Client(\""+QString(getenv("USER"))+"-owl\")\n"
                        "x=client.new_data_vector(\""+QString(_dirfileFilename)+"\",field = \"INDEX\", start=-1, num_frames=1000)\n"
                        "y=client.new_data_vector(\""+QString(_dirfileFilename)+"\",field = \""+QString(padi->source())+"\", start=-1, num_frames=1000)\n"
                        "c=client.new_curve(x,y)\n"
                        "p=client.new_plot((0.5, 0.5), (1.0, 1.0))\n"
                        "p.add(c)\n"
                        );
                        */
    PyRun_SimpleString(QString("import pykst as kst\n"
                       "client = kst.Client(\""+QString(getenv("USER"))+"-owl\")\n"
                       "x=client.new_data_vector(\""+QString(_dirfileFilename)+"\",field = \"INDEX\", start=-1, num_frames=1000)\n"
                       "y=client.new_data_vector(\""+QString(_dirfileFilename)+"\",field = \""+QString(padi->source())+"\", start=-1, num_frames=1000)\n"
                       "c=client.new_curve(x,y)\n"
                       "p=client.new_plot((0.5, 0.5), (1.0, 1.0))\n"
                       "p.add(c)\n"
                       ).toLatin1());
}
