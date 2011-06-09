/* narsil: GUI commanding front-end
 *
 * This software is copyright (C) 2002-2005 University of Toronto
 * 
 * This file is part of narsil.
 * 
 * narsil is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * narsil is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with narsil; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// ***************************************************
// *  Programmed by Adam Hincks                      *
// *                                                 *
// *  Comments on classes & functions in .cpp file   *
// ***************************************************

#ifndef NARSIL_H
#define NARSIL_H

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

extern "C" {
#include "netcmd.h"
}

#include <qvariant.h>
#include <qdialog.h>
#include <qstring.h>
#include <qspinbox.h>
#include <qmainwindow.h>

#include <sys/types.h>

#include <getdata/dirfile.h>

using namespace GetData;

#define SMALL_POINT_SIZE 10
#define LARGE_POINT_SIZE 12

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QButtonGroup;
class QFrame;
class QLabel;
class QLineEdit;
class QListBox;
class QListBoxItem;
class QPushButton;
class QRadioButton;
class QSpinBox;
class QPixmap;
class QCheckBox;
class QComboBox;
class QMultiLineEdit;

class KstFile;
class DoubleEntry;

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
    char (*sdefaults[MAX_N_PARAMS])[32];
};

extern Defaults *defaults;

class MainForm : public QMainWindow
{
  Q_OBJECT

  public:
    MainForm(const char *cf, QWidget* parent = 0, const char* name = 0,
        WFlags fl = 0);
    ~MainForm();

    QFrame *NTopFrame;
    QFrame *NBotFrame;
    QLabel *NCurFileCaption;
    QLineEdit *NCurFile;
    QButtonGroup *NGroupsBox;
    QRadioButton **NGroups;
    QPushButton *NSendButton;
    QPushButton *NSettingsButton;
    QLabel *NSettingsLabel;
    QPushButton *NCurFileButton;
    QPushButton *QuitButton;
    QListBox *NCommandList;
    QLabel *NAboutLabel;
    QLabel *NParamLabels[MAX_N_PARAMS];
    DoubleEntry *NParamFields[MAX_N_PARAMS];
    QLabel *NWaitImage;
    QCheckBox *NVerbose;
    QComboBox *NSendMethod;
    QComboBox *NSendRoute;
    QDialog *NSettingsWindow;
    QMultiLineEdit *NLog;
    QPushButton *NCloseSettingsWindow;

    QWidget *centralWidget;
    QVBoxLayout *theVLayout;
    QHBoxLayout *theHLayout;
    QStatusBar *theStatusBar;

    void TurnOn(void);
    void TurnOff(void);
    QTimer *timer;
    QTimer *ping_timer;

  protected:
    QGridLayout *NGroupsLayout;
    QVBoxLayout *NSettingsLayout;
    QHBoxLayout *NTSettingsLayout;
    QHBoxLayout *NMSettingsLayout;
    QHBoxLayout *NBSettingsLayout;

  private:
    int GroupSIndexes(int group, int *indexes);
    int GroupMIndexes(int group, int *indexes);
    int GetGroup();
    int SIndex(QString cmd);
    int MIndex(QString cmd);
    char *LongestParam();
    void ReadLog(QMultiLineEdit *dest);
    void WriteCmd(QMultiLineEdit *dest, const char *request);
    void WriteErr(QMultiLineEdit *dest, int retstatus);
    void WriteLog(const char *request);
    QLabel* ConnBanner;

    int ReceivePackets(int, int);

    int lastmcmd;
    QString curfile;
    //KstFile *DataSource;
    Dirfile *_dirfile;
    char _curFileName[1024];

    int fid;
    bool sending;

    int verbose;
    int framenum;
    int numframes;
    int dir;
    QPixmap *Images[4];

    public slots:
      void ChangeCommandList();
    void ChooseCommand();
    void Quit();
    void SendCommand();
    void Tick();
    void ChangeCurFile();
    void ShowSettings();
    void Ping();
    void ChangeSettingsLabel();
};
#endif

// vim: ts=2 sw=2 et
