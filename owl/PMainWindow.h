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

#include <QMainWindow>
#include <QDataStream>
#include <QScrollArea>
#include <QSystemSemaphore>
#include <QSettings>
#include "PAbstractDataItem.h"
#include "PObject.h"
#include "PBox.h"
#include "POwlAnimation.h"
#include "PServer.h"

#ifndef PMAINWINDOW_H
#define PMAINWINDOW_H
#define _WINDOW_TITLE_ "Owl 4.0-pre"

struct PTimeDataItem;

class QLineEdit;

namespace Ui {
class PMainWindow;
}

class PMainWindow : public QMainWindow, public PObject
{
    Q_OBJECT
protected:
    QTimer* _ut;
    QObject* _currentObject;
    GetData::Dirfile* _dirfile;
    QString _dirfileFilename;
    PMdiArea* _mdiArea;
    QScrollArea* _scrollArea;
    QString oldStyle;
    QString oldLayout;
    int styleVersion;
    int layoutVersion;
    PStyle* _currowStyle;
    QList<PBox*> _pboxList;
    QList<POwlAnimation*> _owlList;
    bool _dirty;
    PServer* _server;
    bool _deleteScheduled;
    QSettings *_settings;

public:
    friend QDataStream& operator<<(QDataStream&a,PMainWindow&b);
    friend QDataStream& operator>>(QDataStream&a,PMainWindow&b);
    friend QVariant save(PMainWindow&);
    friend void load(QVariant v,PMainWindow&b);
    friend class POwlAnimation;
    friend class PBox;
    friend class PAbstractDataItem;
    friend class PMdiArea;
    static PMainWindow* me;
    explicit PMainWindow(QString file="", QWidget *parent = 0);
    QObject* currentObject() const { return _currentObject; }
    virtual ~PMainWindow();
    void closeEvent(QCloseEvent *);
    bool mouseInactive();

    static QString key;

public slots:
    void readmeHelp();
    void webServerHelp();

    void hideEverything();
    void addPBox();
    void setCurrentObject(PBox*);
    void setCurrentObject(PAbstractDataItem*);
    void setCurrentObject(PNumberDataItem*);
    void setCurrentObject(PMultiDataItem*);
    void setCurrentObject(PTimeDataItem*);
    void setCurrentObject(POwlAnimation*);

    void uiLogic();
    void curfileLogic(bool force = false);
    void setFileLineEditValidity(QLineEdit* fle);
    void newLabelLogic(PAbstractDataItem* padi);
    void resetLink() {curfileLogic(true);}

    void extremaLogic(QString);
    void extremaXHighLogic(double);
    void extremaHighLogic(double);
    void extremaLowLogic(double);
    void extremaXLowLogic(double);

    void showInKst();

    void multiLogic();
    void currowLogic();

    void gdUpdate();
    void serverUpdate();

    void removeCurrentDataItem();
    void obviate(PBox* byeBye);
    void obviate(PAbstractDataItem* byeBye);
    void obviate(POwlAnimation* byeBye);
    void recognizeExtrema(PExtrema*);

    void activate();

    void owlSave();
    void owlSaveAs();
    void owlLoad(QString file="");
    void addOwl();

private:
    Ui::PMainWindow *ui;
};

QDataStream& operator<<(QDataStream&a,PMainWindow&b);
QDataStream& operator>>(QDataStream&a,PMainWindow&b);

#endif//PMAINWINDOW_H
