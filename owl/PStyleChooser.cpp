#include "PStyleChooser.h"
#include "ui_PStyleChooser.h"
#include <QInputDialog>
#include <QColorDialog>

void PStyleNotifier::notifyChange()
{
    if(on) {
        emit change();
        for(int i=0;i<PStyle::_u.size();i++) {
            PStyle::_u[i]->_dirty=0;
        }
    }
}

PStyleNotifier* PStyleNotifier::me=new PStyleNotifier;
PStyle* PStyle::noStyle = 0;
QList<PStyle*> PStyle::_u;

PStyleChooser::PStyleChooser(QWidget *parent) :
    QWidget(parent), current(0),
    ui(new Ui::PStyleChooser)
{
    ui->setupUi(this);
    connect(ui->comboBoxStyle,SIGNAL(currentIndexChanged(QString)),this,SLOT(select(QString)));
    ui->comboBoxStyle->view()->setMinimumWidth(width()*0.8);
    connect(ui->toolButtonB,SIGNAL(toggled(bool)),this,SLOT(boldLogic(bool)));
    connect(ui->toolButtonI,SIGNAL(toggled(bool)),this,SLOT(italicLogic(bool)));
    connect(ui->toolButtonBG,SIGNAL(clicked()),this,SLOT(bgLogic()));
    connect(ui->toolButtonFG,SIGNAL(clicked()),this,SLOT(fgLogic()));
    connect(PStyleNotifier::me,SIGNAL(change()),this,SLOT(refresh()));
}

PStyleChooser::~PStyleChooser()
{
    delete ui;
}

void PStyleChooser::setWidgetStyleRef(PStyle *&style)
{
    current=&style;
}

void PStyleChooser::select(QString s)
{
    Q_ASSERT(current&&(*current));

    if(s=="No Style") {
        ui->toolButtonB->setEnabled(0);
        ui->toolButtonI->setEnabled(0);
        ui->toolButtonBG->setEnabled(0);
        ui->toolButtonFG->setEnabled(0);
        if(*current!=PStyle::noStyle) {
            (*current) = PStyle::noStyle;
        }
    } else if(s=="New Style") {
        bool ok;
        QString n=QInputDialog::getText(this,"New Style","Name",QLineEdit::Normal,"",&ok);

        if(!ok) {
            return;
        }

        (*current) = new PStyle(n,0,0,"white","black",1);
        ui->toolButtonB->setEnabled(1);
        ui->toolButtonI->setEnabled(1);
        ui->toolButtonBG->setEnabled(1);
        ui->toolButtonFG->setEnabled(1);
        PStyleNotifier::me->notifyChange();
        ui->comboBoxStyle->setCurrentIndex(ui->comboBoxStyle->count()-1);
    } else {
        for(int i=0;i<PStyle::_u.size();i++) {
            if(s.contains(PStyle::_u[i]->idText())) {
                if(*current!=PStyle::_u[i]) {
                    (*current) = PStyle::_u[i];
                    ui->toolButtonB->setEnabled(1);
                    ui->toolButtonI->setEnabled(1);
                    ui->toolButtonBG->setEnabled(1);
                    ui->toolButtonFG->setEnabled(1);
                }
            }
        }
    }

    if((*current)->isBold()!=ui->toolButtonB->isChecked()) {
        ui->toolButtonB->setChecked((*current)->isBold());
    }

    if((*current)->isItalic()!=ui->toolButtonI->isChecked()) {
        ui->toolButtonI->setChecked((*current)->isItalic());
    }
}

void PStyleChooser::boldLogic(bool b)
{
    Q_ASSERT(current&&(*current));
    (*current)->setBold(b);
}

void PStyleChooser::italicLogic(bool b)
{
    Q_ASSERT(current&&(*current));
    (*current)->setItalic(b);
}

void PStyleChooser::fgLogic()
{
    Q_ASSERT(current&&(*current));
    QColor c=QColorDialog::getColor((*current)->fgColour());
    if(c.isValid()) {
        (*current)->setFg(c);
    }
}

void PStyleChooser::bgLogic()
{
    Q_ASSERT(current&&(*current));
    QColor c=QColorDialog::getColor((*current)->bgColour());
    if(c.isValid()) {
        (*current)->setBg(c);
    }
}

void PStyleChooser::refresh()
{
    if(!current||!(*current))
    {
        return;
    }
    for(int i=0;i<PStyle::_u.size();i++) {
        bool ok=0;
        for(int j=0;j<ui->comboBoxStyle->count();j++) {
            if(ui->comboBoxStyle->itemText(j).contains(PStyle::_u[i]->idText())) {
                ok=1;
                break;
            }
        }
        if(!ok) {
            ui->comboBoxStyle->addItem(PStyle::_u[i]->name()+" "+PStyle::_u[i]->idText());
        }
    }
    if((*current)==PStyle::noStyle) {
        ui->comboBoxStyle->setCurrentIndex(0);
        ui->toolButtonB->setEnabled(0);
        ui->toolButtonI->setEnabled(0);
        ui->toolButtonBG->setEnabled(0);
        ui->toolButtonFG->setEnabled(0);
    } else for(int i=2;i<ui->comboBoxStyle->count();i++) {
        if(ui->comboBoxStyle->itemText(i).contains((*current)->idText())) {
            ui->comboBoxStyle->setCurrentIndex(i);
            ui->toolButtonB->setEnabled(1);
            ui->toolButtonI->setEnabled(1);
            ui->toolButtonBG->setEnabled(1);
            ui->toolButtonFG->setEnabled(1);
            break;
        }
    }
}
