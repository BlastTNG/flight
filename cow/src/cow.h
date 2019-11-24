/* cow (previously known as narsil): GUI commanding front-end
 *
 * This software is copyright (C) 2002-2005 University of Toronto
 *
 * This file is part of cow.
 *
 * cow is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * cow is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with cow; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// ***************************************************
// *  Programmed by Adam Hincks                      *
// *  Later poked at a bit by D.V.Wiebe              *
// *  further desecrated by cbn                      *
// *  Commanding hacked to hell by M.D.P.Truch       *
// *  "Ported" to qt4 by drmrshdw                    *
// ***************************************************

#ifndef COW_H
#define COW_H

extern "C" {
#include "netcmd.h"
}

#include <QVariant>
#include <QDialog>
#include <QString>
#include <QSpinBox>
#include <QMainWindow>
#include <QLineEdit>
#include <limits>
#include <limits.h>
#include <QMap>

#include <sys/types.h>

#include <getdata/dirfile.h>

using namespace GetData;

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QTextEdit;
class QGroupBox;
class CowOmniBox;
class QFrame;
class QLabel;
class QLineEdit;
class QListWidget;
class QListBoxItem;
class QPushButton;
class QRadioButton;
class QSpinBox;
class QPixmap;
class QCheckBox;
class QComboBox;
class QMultiLineEdit;

class KstFile;
class CowDoubleEntry;

class Defaults
{
public:
    Defaults();
    void Set(int, int, QString);
    int asInt(int, int);
    double asDouble(int, int);
    const char* asString(int, int);
    void Save();

private:
    double *rdefaults[MAX_N_PARAMS];
    int *idefaults[MAX_N_PARAMS];
    char (*sdefaults[MAX_N_PARAMS])[CMD_STRING_LEN];
};

extern Defaults *defaults;

class MainForm : public QMainWindow
{
    Q_OBJECT

public:
    MainForm(const char *cf, const QString &herdfile, int link, QWidget* parent = 0, const char* name = 0,
             Qt::WindowFlags fl = 0);
    ~MainForm();

    QFrame *NTopFrame;
    QFrame *NBotFrame;
    QLineEdit *NCurFile;
    QPushButton *NHost;

    QTabWidget *TabWidget;
    QWidget *tab1;
    QGridLayout *tab1Layout;

    QWidget *tab2;
    QGridLayout *tab2Layout;

    QGroupBox *NGroupsBox;
    QRadioButton **NGroups;

    QGroupBox *HerdGroupBox;
    QRadioButton **HerdGroups;

    QPushButton *NSendButton;
    //QLabel *NSettingsLabel;
    QPushButton *QuitButton;
    CowOmniBox* NOmniBox;
    QListWidget *NCommandList;
    QLabel *NAboutLabel;
    QLabel *NParamLabels[MAX_N_PARAMS];
    QWidget *NParamFields[MAX_N_PARAMS];
    QLabel *NWaitImage;
    QCheckBox *NVerbose;
    QComboBox *NSendMethod;
    QComboBox *NSendRoute;
    QTextEdit *NLog;
    //QPushButton *NCloseSettingsWindow;

    QWidget *centralWidget;
    QVBoxLayout *theVLayout;
    QHBoxLayout *theHLayout;
    QStatusBar *theStatusBar;

    void TurnOn(void);
    void TurnOff(void);
    QTimer *timer;
    QTimer *ping_timer;

public:
    struct OmniPair
    {
        QString name;
        int group;
    };
protected:

    QList<OmniPair> OmniList;
    QGridLayout *NGroupsLayout;
    QGridLayout *HerdGroupsLayout;

    void keyPressEvent(QKeyEvent *);

private:
    void PopulateOmnibox();
    int GroupSIndexes(int group, int *indexes);
    int GroupMIndexes(int group, int *indexes);
    int GetGroup();
    int LinkChanged();
    int SIndex(QString cmd);
    int MIndex(QString cmd);
    char *LongestParam();
    void ReadLog(QTextEdit *dest);
    void WriteCmd(QTextEdit *dest, const char *request);
    void WriteErr(QTextEdit *dest, const char *message, int retstatus);
    void WriteErr(QTextEdit *dest, int retstatus);
    void WriteLog(const char *request);

    void ReadHerdFile(const QString &herdfile);

    QStringList ListOfHerds;
    QHash<QString, QStringList> HerdHash;

    QLabel* ConnBanner;

    int ReceivePackets(int, int);

    int lastmcmd;
    QString curfile;
    Dirfile *_dirfile;

    int fid;
    bool sending;

    int pong;
    int verbose;
    int framenum;
    int numframes;
    int dir;
    QPixmap *Images[6];

    QMap<int,int> index_defaults;

public slots:
    void OmniParse(QString filter="__AUTODETECT__");
    void OmniSync();
    void ChangeCommandList(bool really=true);
    void IndexComboChanged(int i) {ChooseCommand(true, i);}
    void ChooseCommand(bool index_combo_changed=false, int combo_index = 0);
    void Quit();
    void SendCommand();
    void Tick();
    void Ping();
    void ChangeHost();
    void ServerDropped();
    void nOmniBox_completerActivated(const QString & text);
    void nOmniBox_textEdited(const QString & text);
    //void testTextChanged(const QString & text);       //for debugging
    //void testCursorPositionChanged(int o, int n);       //for debugging
};

#endif

// vim: ts=2 sw=2 et
