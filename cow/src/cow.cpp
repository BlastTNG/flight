/* cow (previously known as narsil): GUI commanding front-end
 *
 * This software is copyright (C) 2002-2011 University of Toronto
 * Parts of this software are copyright 2010 Matthew Truch
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
// *  "Ported" to qt4 by Joshua Netterfield          *
// *  Herds added by cbn                             *
// ***************************************************

#define _XOPEN_SOURCE 501
#define _POSIX_C_SOURCE 200809L

#include <QCompleter>
#include <QDebug>
#include <QApplication>
#include <QButtonGroup>
#include <QGroupBox>
#include <QDir>
#include <QFileDialog>
#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QLayout>
#include <QVariant>
#include <QToolTip>
#include <QWhatsThis>
#include <QSpinBox>
#include <QValidator>
#include <QPixmap>
#include <QCheckBox>
#include <QComboBox>
#include <QMessageBox>
#include <QTextEdit>
#include <QTimer>
#include <QFile>
#include <QDateTime>
#include <QSizePolicy>
#include <QMainWindow>
#include <QStatusBar>
#include <QDoubleSpinBox>
#include <QListWidget>
#include <QTextStream>
#include <QInputDialog>
#include <QSettings>
#include <QDirModel>
#include <QKeyEvent>
#include <QStyleFactory>
#include <QTabWidget>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <signal.h>
#include <time.h>
#include <pwd.h>

#include "widgets.h"
#include "cow.h"

#ifndef DATA_ETC_COW_DIR
#error Edit cow.pro to define DATA_ETC_COW_DIR !
#endif

#define PONG_MISS_MAX 5

#define PADDING 3
#define SPACING 3

#define LOGFILE DATA_ETC_COW_DIR "/log.txt"
#define LOGFILEDIR DATA_ETC_COW_DIR "/log/"
#define HERDFILEDIR "/data/etc/cow/"

#define DATA_DIR DATA_ETC_COW_DIR

#define PING_TIME 19000
#define WAIT_TIME 80

#ifdef __APPLE__
/* On a Mac keyboard, ⌘ (Command) is represented by ControlModifier, and
 * Control by MetaModifier
 */
#define SEND_KEY ev->key() == Qt::Key_Enter && (ev->modifiers() & Qt::ControlModifier)
/* For reasons of lameness, SEND_KEY_NAME must be a particular length */
#define SEND_KEY_NAME "Cmd-Ent"
#else
#define SEND_KEY ev->key() == Qt::Key_F12 && (ev->modifiers()&Qt::ShiftModifier)
#define SEND_KEY_NAME "Shift-F12"
#endif

QString blastcmd_host;
QString herdfile;
int stop_comms;
int missed_pongs;

/* Defaults class holds the default parameter values */
Defaults::Defaults()
{
  int i, j;
  int fp;
  int n_read = 0;

  for (i = 0; i < MAX_N_PARAMS; ++i) {
    rdefaults[i] = (double*)malloc(sizeof(double) * client_n_mcommands);
    idefaults[i] = (int*)malloc(sizeof(int) * client_n_mcommands);
    sdefaults[i] = (char(*)[CMD_STRING_LEN])malloc(sizeof(char) *
        CMD_STRING_LEN * client_n_mcommands);
  }

  /* Read in previous default values */
  if ((fp = open(DATA_DIR "/prev_status", O_RDONLY)) >= 0) {
    for (i = 0; i < MAX_N_PARAMS; ++i) {
      n_read += read(fp, rdefaults[i], sizeof(double) * client_n_mcommands);
      n_read += read(fp, idefaults[i], sizeof(int) * client_n_mcommands);
      n_read += read(fp, sdefaults[i], sizeof(char) * CMD_STRING_LEN
          * client_n_mcommands);
    }
    close(fp);
  }

  if (n_read != (int)(sizeof(int) + sizeof(double) + 32 * sizeof(char))
      * client_n_mcommands * MAX_N_PARAMS)
    for (i = 0; i < client_n_mcommands; i++)
      for (j = 0; j < MAX_N_PARAMS; j++)
        rdefaults[j][i] = idefaults[j][i] = sdefaults[j][i][0] = 0;
}

//-------------------------------------------------------------
//
// Defaults::Save: Cow remembers any values entered for the
//      next time it starts up
//
//-------------------------------------------------------------

void Defaults::Save() {
  int fp, i;

  /* Write file with defaults */
  fp = open(DATA_DIR "/prev_status", O_WRONLY|O_CREAT|O_TRUNC, 0666);
  if (fp < 0)
    printf("Warning: could not open prev_status file\n");
  else {
    for (i = 0; i < MAX_N_PARAMS; ++i) {
      if (write(fp, rdefaults[i], sizeof(double) * client_n_mcommands) < 0)
        printf("Warning: could not write rdefults to prev_status file\n");
      if (write(fp, idefaults[i], sizeof(int) * client_n_mcommands) < 0)
        printf("Warning: could not write idefults to prev_status file\n");
      if (write(fp, sdefaults[i], sizeof(char) * CMD_STRING_LEN
            * client_n_mcommands) < 0)
        printf("Warning: could not write sdefults to prev_status file\n");
    }
    close(fp);
  }
}

void Defaults::Set(int i, int j, QString text)
{
  idefaults[j][i] = text.toInt();
  rdefaults[j][i] = text.toDouble();
  strncpy(sdefaults[j][i], text.toLatin1(), CMD_STRING_LEN - 1);
  sdefaults[j][i][CMD_STRING_LEN - 1] = 0;
}

int Defaults::asInt(int i, int j) { return idefaults[j][i]; }
double Defaults::asDouble(int i, int j) { return rdefaults[j][i]; }
const char* Defaults::asString(int i, int j) { return sdefaults[j][i]; }

//***************************************************************************
//****     CLASS MainForm -- main control class
//***************************************************************************

//-------------------------------------------------------------
// GetGroup (private): checks the radio group of buttons to see
//      which group is checked
//
//   Returns: index of the selected group
//-------------------------------------------------------------

int MainForm::GetGroup() {
  int i;

  if (TabWidget->currentIndex()==0) {
    for (i = 0; i < client_n_groups; i++) {
      if (NGroups[i]->isChecked()) {
        return i;
      }
    }
  } else {
    for (i=0; i<ListOfHerds.length(); i++) {
      if (HerdGroups[i]->isChecked()) {
        return i;
      }
    }
  }

  return 0;
}

int TheSort(const void* a, const void* b)
{
  const char *za = (*(int*)a >= client_n_scommands) ? client_mcommands[*(int*)a
    - client_n_scommands].name : client_scommands[*(int*)a].name;
  const char *zb = (*(int*)b >= client_n_scommands) ? client_mcommands[*(int*)b
    - client_n_scommands].name : client_scommands[*(int*)b].name;
  return strcmp(za, zb);
}

void MainForm::OmniParse(QString x) //evil, evil function (-Joshua)
{
  x=(x=="__AUTODETECT__")?NOmniBox->text():x;
  QString r=x;
  if(r.contains(' ')) r.truncate(x.indexOf(' ')+1);

  bool special=0;

  if (NOmniBox->hasFocus()) {
    TabWidget->setCurrentIndex(0);
  }

  if(!NOmniBox->hasFocus()||((r==x||x+" "==r)&&NOmniBox->oldXSize<x.size())) {
    special=1;
  }
  if(NOmniBox->hasFocus()) NOmniBox->oldXSize=x.size();

  //search for command, and select it if matched
  int best_i=-1;
  for(int i=0;i<OmniList.size();i++) {
    if(r==OmniList[i].name+' ') {
      if(!NGroups[OmniList[i].group]->isChecked()) {
        best_i=i;
      } else {
        best_i=-1;
        break;
      }
    }
  }
  if (TabWidget->currentIndex()==0) {
    if(best_i!=-1) {
      NGroups[OmniList[best_i].group]->toggle();
    }
  }

  for(int i=0;i<NCommandList->count();i++) {
    if(NCommandList->item(i)->text()+" "==r) {
      NCommandList->setCurrentRow(i);
    }
  }

  //look for command in current list, then set box colour and dis/enable Send button
  bool ok=0;
  for(int i=0;i<NCommandList->count();i++) {
    if(r==NCommandList->item(i)->text()+' ') {
      ok=1;
      break;
    }
  }
  QPalette f = NOmniBox->palette();
  if(!ok||x.contains("  ")) {
    f.setColor(QPalette::Base,"pink");
    NSendButton->setEnabled(0);
  } else {    //moved from questionably useful part above
    f.setColor(QPalette::Base,"white");
    NSendButton->setEnabled(1);
  }
  NOmniBox->setPalette(f);
  //FIXME after selecting a group, commands get erased not completed eg click "HWPR", type "stop"


  if(x.contains(' ')) {
    if(special && ok) {
      x.truncate(x.indexOf(' ')+1);
      for(int i=0;i<MAX_N_PARAMS;i++) {
        if(NParamFields[i]->isVisible()) {
          x.append(i?" ":"");
          x.append(dynamic_cast<AbstractCowEntry*>(NParamFields[i])->Text());
        }
      }

      int cp=NOmniBox->cursorPosition();
      int sbeg=NOmniBox->selectionStart();
      int slen=NOmniBox->selectedText().size();
      if(x!=NOmniBox->text()) {
        NOmniBox->setRealText(x);
      }
      NOmniBox->setCursorPosition(cp);
      if(sbeg!=-1) {
        NOmniBox->setSelection(sbeg,slen);
      }
    }

    QStringList words=x.split(" ");
    int max=MAX_N_PARAMS+1;
    for(int i=1;i<=MAX_N_PARAMS;i++) {
      if(!NParamFields[i-1]->isVisible()) {
        max=i;
        break;
      }
      if(words.size()<=i||words[i]=="") {
        QPalette f = NOmniBox->palette();
        f.setColor(QPalette::Base,"pink");
        NOmniBox->setPalette(f);
        NSendButton->setEnabled(0);
        break;
      }
      if(NParamFields[i-1]->isVisible() && !words[i].isEmpty()) {
        // words[i].replace("..","."); // doesn't seem to do anything...
        bool ok=dynamic_cast<CowStringEntry*>(NParamFields[i-1]);
        if(ok) {
          int j=i;
          while(++j<words.size()) {
            words[i]+=" "+words[j];
          }
        }
        if(!ok) {
          words[i].toDouble(&ok);
        }
        if(!ok) {
          words[i].toInt(&ok);
        }
        if(ok) {
          if(dynamic_cast<CowStringEntry*>(NParamFields[i-1])) {
            dynamic_cast<AbstractCowEntry*>(NParamFields[i-1])->SetStringValue(words[i]);
          } else if(dynamic_cast<AbstractCowEntry*>(NParamFields[i-1])->Text().toFloat()!=words[i].toFloat()){
            dynamic_cast<AbstractCowEntry*>(NParamFields[i-1])->SetStringValue(QString::number(words[i].toFloat()));
          }
        }
      }
    }
    if(words.size()&&words.back()=="") words.pop_back();
    if(words.size()<max) {
      QPalette f = NOmniBox->palette();
      f.setColor(QPalette::Base,"pink");
      NOmniBox->setPalette(f);
      NSendButton->setEnabled(0);
    }

  }
}

void MainForm::nOmniBox_completerActivated(const QString & text)
{
  //always special when selected from completer
  NOmniBox->oldXSize = 0;

  OmniParse(text);

}

/*
   void MainForm::testTextChanged(const QString & text)
   {
   qDebug() << "test textChanged: " << NOmniBox->text() << " -> " << text;
   }

   void MainForm::testCursorPositionChanged(int o, int n)
   {
   qDebug() << "cursor moved: " << o << " -> " << n;
   }
   */

void MainForm::nOmniBox_textEdited(const QString & text)
{
  int cp=NOmniBox->cursorPosition();
  int sbeg=NOmniBox->selectionStart();
  int slen=NOmniBox->selectedText().size();
  NOmniBox->setRealText(text);
  NOmniBox->setCursorPosition(cp);
  if(sbeg!=-1) {
    NOmniBox->setSelection(sbeg,slen);
  }
  OmniParse(text);
  NOmniBox->setFocus();
}

void MainForm::OmniSync()
{
  QString x=NOmniBox->text();
  QStringList words=x.split(" ");
  for(int i=1;i<words.size();i++) {
    if((i<MAX_N_PARAMS) && NParamFields[i-1]->isVisible() && !words[i].isEmpty()) {
      // words[i].replace("..","."); // doesn't seem to do anything...
      words[i]=dynamic_cast<AbstractCowEntry*>(NParamFields[i-1])->Text();
    }
  }

  if(NOmniBox->hasFocus()) return;

  int cp=NOmniBox->cursorPosition();
  int sbeg=NOmniBox->selectionStart();
  int slen=NOmniBox->selectedText().size();
  NOmniBox->setRealText(words.join(" "));
  NOmniBox->setCursorPosition(cp);
  if(sbeg!=-1) {
    NOmniBox->setSelection(sbeg,slen);
  }
  OmniParse();
}

//-------------------------------------------------------------
// ChangeCommandList (slot): when a new group is selected,
//      the list of commands must be cleared and the new
//      group's commands added.  Triggered when a group button
//      is selected.
//-------------------------------------------------------------

void MainForm::ChangeCommandList(bool really) {
  if(!really) {
    return;
  }

  int indexes[client_n_scommands + client_n_mcommands];
  int i;
  int max;

  NCommandList->clearSelection();
  NCommandList->clearFocus();
  NCommandList->clear();

  if (TabWidget->currentIndex()==0) {
    max = GroupSIndexes(GetGroup(), indexes);
    max += GroupMIndexes(GetGroup(), &indexes[max]);
    //fprintf(stderr, "Group count: %i\n", max);
    qsort(indexes, max, sizeof(int), &TheSort);

    for (i = 0; i < max; i++)
      if (indexes[i] >= client_n_scommands) {
        NCommandList->addItem(client_mcommands[indexes[i]
            - client_n_scommands].name);
      } else {
        NCommandList->addItem(client_scommands[indexes[i]].name);
      }
  } else {
    if (ListOfHerds.size() > 0) {
      NCommandList->addItems(HerdHash[ListOfHerds.at(GetGroup())]);
      NCommandList->setSelectionMode(QAbstractItemView::SingleSelection);
    }
  }
  ChooseCommand();
  //    OmniParse();
}

// Check the dirfile name, and see if it's chagned */
int MainForm::LinkChanged(void)
{
  static char old_target[4096] = { 0 };
  ssize_t n;

  char lnkname[4096];
  char target[4096];
  strncpy(lnkname, NCurFile->text().toStdString().c_str(), 4096);
  lnkname[4095] = 0;

  n = readlink(lnkname, target, 4096);

  if (n < 0) {
    //qDebug() << "readlink returned error";
    old_target[0] = 0; /* forget old name */
    return 1; // assume we need to re-read
  }

  if (strncmp(target, old_target, n)) {
    // Link changed
    strncpy(old_target, target, n);
    old_target[n] = 0;
    return 1;
  }

  // No change.
  return 0;
}

//-------------------------------------------------------------
// ChooseCommand (slot): triggered when a command is selected
//      from the list. Changes labels and shows spin-boxes
//      as appropiate for the command.
//-------------------------------------------------------------

void MainForm::ChooseCommand(bool index_combo_changed, int combo_index) {
  int i, index;
  double indata;
  Q_UNUSED(combo_index);

  if (LinkChanged()) {
    delete _dirfile;
    _dirfile = new Dirfile(NCurFile->text().toStdString().c_str(), GD_RDONLY);
    if (_dirfile->Error()) {
      delete _dirfile;
      _dirfile = NULL;
    }
  }

  // Remember parameter values
  if (lastmcmd != -1) {
    for (i = 0; i < client_mcommands[lastmcmd].numparams; i++)
      dynamic_cast<AbstractCowEntry*>(NParamFields[i])->RecordDefaults();

    defaults->Save();
  }

  // It can happen that this function be called with nothing selected
  if (!NCommandList->currentIndex().isValid() ||
       (!index_combo_changed && (!NCommandList->hasFocus()&&!NOmniBox->hasFocus()))) {
    NSendButton->setDisabled(true);
    lastmcmd = -1;
    NAboutLabel->setText(tr("No command selected."));
    for (i = 0; i < MAX_N_PARAMS; i++) {
      NParamLabels[i]->hide();
      NParamFields[i]->hide();
    }
  } else {
    NSendButton->setEnabled(true);
    if(!NOmniBox->text().contains(NCommandList->currentItem()->text()+" ")) {
      NOmniBox->setRealText(NCommandList->currentItem()->text()+" ");
    }
    NOmniBox->setCursorPosition(NOmniBox->text().size());
    if ((index = SIndex( NCommandList->currentItem()->text()) ) != -1) {
      // Set up for a single command
      NAboutLabel->setText(client_scommands[index].about);
      lastmcmd = -1;
      for (i = 0; i < MAX_N_PARAMS; i++) {
        NParamLabels[i]->hide();
        NParamFields[i]->hide();
      }
    } else if ((index = MIndex(NCommandList->currentItem()->text()))
        != -1) {
      // Set up for a multi command -- show the parameter spin boxes
      NAboutLabel->setText(client_mcommands[index].about);
      lastmcmd = index;

      //bool IsData = DataSource->update();

      int index_serial = 0;
      for (i = 0; i < MAX_N_PARAMS; i++) {
        if (i < client_mcommands[index].numparams) {

          NParamLabels[i]->setText(client_mcommands[index].params[i].name);
          NParamLabels[i]->show();

          bool typeChanged=0;
          QRect geometry=NParamFields[i]->geometry();
          if (!(dynamic_cast<CowStringEntry*>(NParamFields[i])) && client_mcommands[index].params[i].type=='s') {
            typeChanged=1;
            delete NParamFields[i];
            connect(NParamFields[i]=new CowStringEntry(NTopFrame,"NParamLabels"),
                SIGNAL(textEdited(QString)),this,SLOT(OmniSync()));
          } else if (client_mcommands[index].params[i].nt) {
            CowComboEntry* cce = dynamic_cast<CowComboEntry*>(NParamFields[i]);
            if (cce) {
              if (client_mcommands[index].params[i].index_serial) {
                disconnect(cce, SIGNAL(activated(int)), this, SLOT(IndexComboChanged(int)));
              }
              cce->clear();
            } else {
              typeChanged=1;
              delete NParamFields[i];
              cce=new CowComboEntry(NTopFrame,"NParamLabels");
            }
            if (client_mcommands[index].params[i].index_serial) {
              connect(cce, SIGNAL(activated(int)), this, SLOT(IndexComboChanged(int)));
            }
            NParamFields[i] = cce;
            for (int i_par = 0; client_mcommands[index].params[i].nt[i_par] != 0; i_par++) {
              cce->addItem(client_mcommands[index].params[i].nt[i_par]);
            }
            cce->minVal = client_mcommands[index].params[i].min;
            connect(dynamic_cast<CowComboEntry*>(NParamFields[i]),
                    SIGNAL(valueEdited()),this,SLOT(OmniSync()));
          } else if (!(dynamic_cast<CowDoubleEntry*>(NParamFields[i])) && client_mcommands[index].params[i].type!='s'
                     && client_mcommands[index].params[i].nt == 0) {
            typeChanged=1;
            if (NParamFields[i]->hasFocus()) {
              qDebug() << "attempt to delete double entry which had focus! Leaking memory instead...";
              NParamFields[i]->hide();
            } else {
              delete NParamFields[i];
            }
            NParamFields[i]=new CowDoubleEntry(NTopFrame,"NParamLabels");
            connect(dynamic_cast<CowDoubleEntry*>(NParamFields[i]),
                SIGNAL(valueEdited()),this,SLOT(OmniSync()));
          }
          if(typeChanged) {
            int w2 = NParamLabels[i]->width();

            NParamFields[i]->setFixedWidth(w2/2);
            NParamFields[i]->adjustSize();

            NParamFields[i]->setGeometry(geometry);
          }
          NParamFields[i]->show();
          if (i > 0) {
            setTabOrder(NParamFields[i-1], NParamFields[i]);
          } else {
            setTabOrder(NCommandList, NParamFields[i]);
          }
          dynamic_cast<AbstractCowEntry*>(NParamFields[i])->SetParentField(index, i);
          dynamic_cast<AbstractCowEntry*>(NParamFields[i])->SetType(client_mcommands[index].params[i].type);
          if (client_mcommands[index].params[i].type == 's') {
            dynamic_cast<AbstractCowEntry*>(NParamFields[i])->SetStringValue(
                client_mcommands[index].params[i].field);
          } else {
            int nf;
            if (client_mcommands[index].params[i].index_serial) {
              int i_s = client_mcommands[index].params[i].index_serial;
              CowComboEntry *cce=dynamic_cast<CowComboEntry*>(NParamFields[i]);
              if (cce) {
                if (index_combo_changed) {
                  cce->SetDefaultValue(index, i);
                  index_defaults.insert(i_s, cce->Text().toInt());
                } else {
                  cce->SetIndex(index_defaults.value(i_s, client_mcommands[index].params[i].min) - client_mcommands[index].params[i].min);
                  cce->RecordDefaults();
                  cce->SetDefaultValue(index, i);
                }
              }
            } else if (_dirfile && ((nf = _dirfile->NFrames())>0) &&
                (_dirfile->GetData( client_mcommands[index].params[i].field,
                                    nf-1, 0, 0, 1, // 1 sample from frame nf-1
                                    Float64, (void*)(&indata))!=0)) {
              dynamic_cast<AbstractCowEntry*>(NParamFields[i])->SetValue(indata);
            } else {
              double d;
              char cmdstr[SIZE_CMDPARNAME];
              if (index_serial) {
                sprintf(cmdstr, "%s;%d;%s", client_mcommands[index].name,
                    index_serial, client_mcommands[index].params[i].name);
              } else {
                // FIXME: index parameter
                sprintf(cmdstr, "%s;%s", client_mcommands[index].name,
                    client_mcommands[index].params[i].name);
              }

              if (stop_comms)
                d = DEF_NOT_FOUND;
              else if (NetCmdGetDefault(&d, cmdstr)) {
                if (missed_pongs < PONG_MISS_MAX - 1)
                  missed_pongs++;
                d = DEF_NOT_FOUND;
              }


              if ((d == DEF_NOT_FOUND) ||
                  (client_mcommands[index].params[i].index_serial))
              {
                dynamic_cast<AbstractCowEntry*>(NParamFields[i])->SetDefaultValue(index, i);
              } else {
                dynamic_cast<AbstractCowEntry*>(NParamFields[i])->SetValue(d);
              }
            }
            if (client_mcommands[index].params[i].index_serial>0) {
              index_serial = dynamic_cast<AbstractCowEntry*>(NParamFields[i])->Text().toInt();
            }
          }
        } else {
          NParamLabels[i]->hide();
          NParamFields[i]->hide();
        }
      }
    }
    if (!NOmniBox->hasFocus()) {
      OmniParse();
    }
    //NOmniBox->setFocus();
    NOmniBox->setCursorPosition(NOmniBox->text().indexOf(" ")+1);
  }
}


//-------------------------------------------------------------
//
// GroupSIndexes (private): creates a list of references to
//      single commands in the same group
//
//   group: the group the commands should belong to
//   *indexes: array to build the list in
//
//   Returns: number of single commands in the group
//
//-------------------------------------------------------------

int MainForm::GroupSIndexes(int group, int *indexes) {
  int i;
  int num = 0;

  for (i = 0; i < client_n_scommands; i++)
    if (client_scommands[i].group & (1 << group))
      indexes[num++] = i;

  return num;
}


//-------------------------------------------------------------
//
// GroupMIndexes (private): creates a list of references to
//      multiple commands in the same group
//
//    group: the group the commands should belong to
//    *indexes: array to build the list in
//
//    Returns: number of multiple commands in the group
//
//-------------------------------------------------------------

int MainForm::GroupMIndexes(int group, int *indexes) {
  int i;
  int num = 0;

  for (i = 0; i < client_n_mcommands; i++)
    if (client_mcommands[i].group & (1 << group))
      indexes[num++] = i + client_n_scommands;

  return num;
}


//-------------------------------------------------------------
//
// SIndex (private): given the command, retrieves the
//      index of the single command
//
//   cmd: command name
//
//   Returns: command index; -1 if not found
//
//-------------------------------------------------------------

int MainForm::SIndex(QString cmd) {
  int i;

  for (i = 0; i < client_n_scommands; i++) {
    if (strcmp(client_scommands[i].name, cmd.toLatin1()) == 0)
      return i;
  }

  return -1;
}


//-------------------------------------------------------------
//
// MIndex (private): given the command, retrieves the
//      index of the multiple command
//
//   cmd: command name
//
//   Returns: command index; -1 if not found
//
//-------------------------------------------------------------

int MainForm::MIndex(QString cmd) {
  int i;

  for (i = 0; i < client_n_mcommands; i++) {
    if (strcmp(client_mcommands[i].name, cmd.toLatin1()) == 0)
      return i;
  }

  return -1;
}


//-------------------------------------------------------------
//
// LongestParam (private): looks through all the parameters
//      from all the multiple commands for the longest
//
//   Returns: length of the longest parameter
//
//-------------------------------------------------------------

char *MainForm::LongestParam() {
  unsigned int i;
  int j;
  unsigned int len = 0;
  static char lp[120];

  for (i = 0; i < client_n_mcommands; i++) {
    for (j = 0; j < client_mcommands[i].numparams; j++) {
      if (strlen(client_mcommands[i].params[j].name) > len) {
        len = strlen(client_mcommands[i].params[j].name);
        strcpy(lp, client_mcommands[i].params[j].name);
      }
    }
  }

  return lp;
}


//-------------------------------------------------------------
//
// Quit (slot): triggered when user presses quit button
//
//-------------------------------------------------------------

void MainForm::Quit() {
  // Settings should be sticky
  {
        QSettings settings("University of Toronto","cow");
        settings.setValue("blastcmd_host",QString(blastcmd_host));
        settings.setValue("curfile",NCurFile->text());
        settings.setValue("link",NSendMethod->currentIndex());
        settings.setValue("geometry", saveGeometry());
  }

  int i;

  // Remember parameter values
  if (lastmcmd != -1) {
    for (i = 0; i < client_mcommands[lastmcmd].numparams; i++)
      dynamic_cast<AbstractCowEntry*>(NParamFields[i])->RecordDefaults();

    defaults->Save();
  }
}


//-------------------------------------------------------------
//
// Tick (slot): triggered by timer.
//      Animate the cool picture of THE COW;
//      Check for incoming messages and do as appropriate.
//
//-------------------------------------------------------------

void MainForm::Tick() {
  timer->stop();

  if (framenum) {
    framenum+=dir;
    if(framenum==numframes-1) dir=-1;
    NWaitImage->setPixmap(*Images[framenum]);
  }

  if (!stop_comms) {
    ReceivePackets(!verbose, CMD_NONE);

    ConnBanner->setText(NetCmdBanner());
  }
  timer->start(WAIT_TIME);
}

int MainForm::ReceivePackets(int silent, int wait_for)
{
  Q_UNUSED(silent);
  int returnstatus;
  char oob_message[1024];

  do {
    returnstatus = NetCmdReceive(!verbose, 1024, oob_message);
    if ((returnstatus & 0xFF) == CMD_PING) {
      pong = 1;
      if (missed_pongs > 0) {
        printf("Got a PONG from server.\n");
        missed_pongs = 0;
      }
    } else if (sending && ((returnstatus & 0xFF) == CMD_BCMD)) {
      if ((returnstatus >> 8) == 12)
        returnstatus = (99 << 8);

      if (oob_message[0])
        WriteErr(NLog, oob_message, returnstatus >> 8);
      else
        WriteErr(NLog, NULL, returnstatus >> 8);

      TurnOff();
    } else if ((returnstatus & 0xff) == CMD_ERRR)
      break;
  } while (returnstatus != CMD_NONE && returnstatus != wait_for && !stop_comms);
  return returnstatus;
}

void MainForm::Ping(void) {
  ping_timer->stop();
  if (!pong) { /* missed a pong */
    printf("Missed PONG #%i from server!\n", missed_pongs);
    if (++missed_pongs > PONG_MISS_MAX) {
      stop_comms = 1;
      ServerDropped();
    }
  }

  if (!NetCmdPing()) {
    /* ping succesfully sent */
    pong = 0; /* wait for next pong */
  }

  ping_timer->start(PING_TIME);
}


//-------------------------------------------------------------
//
// TurnOn (private): called when the user tries to send a
//      command.  Most user input is suspended while the
//      command is being send.
//
//-------------------------------------------------------------

void MainForm::TurnOn(void) {

  sending = true;

  for(int i=0;i<centralWidget->children().size();i++) {
    QWidget* c=dynamic_cast<QWidget*>(centralWidget->children()[i]);
    if(c) c->setEnabled(0);
  }
  for(int i=0;i<NBotFrame->children().size();i++) {
    QWidget* c=dynamic_cast<QWidget*>(NBotFrame->children()[i]);
    if(c) c->setEnabled(0);
  }
  centralWidget->setEnabled(1);
  NBotFrame->setEnabled(1);
  NSendButton->setEnabled(1);

  NSendButton->setText(tr("Cancel"));
}


//-------------------------------------------------------------
//
// TurnOff (private): called when the command has been sent,
//      or user has cancelled
//
//-------------------------------------------------------------

void MainForm::TurnOff(void) {
  sending = false;

  for(int i=0;i<centralWidget->children().size();i++) {
    QWidget* c=dynamic_cast<QWidget*>(centralWidget->children()[i]);
    if(c) c->setEnabled(1);
  }
  for(int i=0;i<NBotFrame->children().size();i++) {
    QWidget* c=dynamic_cast<QWidget*>(NBotFrame->children()[i]);
    if(c) c->setEnabled(1);
  }
  NSendButton->setText(tr("Send (" SEND_KEY_NAME ")"));
}


//-------------------------------------------------------------
//
// SendCommand (slot): triggered when user presses send button.
//
//-------------------------------------------------------------

void MainForm::SendCommand() {
  int index, j;
  char buffer[1024];
  char request[1024];
  char sparam[CMD_STRING_LEN];
  const char link[] = "LTIP";
  const char route[] = "12";

  if (!sending) {

    // Select appropiate flags
    verbose = NVerbose->isChecked();

    request[0] = link[NSendMethod->currentIndex()];
    request[1] = route[NSendRoute->currentIndex()];
    request[2] = ' ';

    // command name
    strcpy(request + 3, NCommandList->currentItem()->text().toLatin1());

    // Parameters
    if ((index = MIndex(NCommandList->currentItem()->text())) != -1)
    {

      // Check to see if this command requires a confirm
      if (client_mcommands[index].group & CONFIRM) {
        sprintf(buffer, "The command %s requires confirmation.\n"
            "Are you sure you want to send this command?",
            NCommandList->currentItem()->text().toStdString().c_str());

        if ( QMessageBox::warning(this, "Confirm Command", tr(buffer),
              QMessageBox::Yes, QMessageBox::Escape | QMessageBox::No |
              QMessageBox::Default) == QMessageBox::No ) {
          WriteCmd(NLog, request);
          WriteErr(NLog, 11);
          return;
        }
      }

      for (j = 0; j < client_mcommands[index].numparams; j++) {
        dynamic_cast<AbstractCowEntry*>(NParamFields[j])->RecordDefaults();
        if (defaults->asDouble(index, j) < client_mcommands[index].params[j].min) {
          sprintf(buffer, "Error: Parameter \"%s\" out of range: %f < %f",
                  client_mcommands[index].params[j].name,
                  defaults->asDouble(index, j),
                  client_mcommands[index].params[j].min);
          WriteCmd(NLog, buffer);
          WriteErr(NLog, 10);
          return;
        } else if (defaults->asDouble(index, j) > client_mcommands[index].params[j].max) {
          sprintf(buffer, "Error: Parameter \"%s\" out of range: %f > %f",
                  client_mcommands[index].params[j].name,
                  defaults->asDouble(index, j),
                  client_mcommands[index].params[j].max);
          WriteCmd(NLog, buffer);
          WriteErr(NLog, 10);
          return;
        }
        if (client_mcommands[index].params[j].type == 'i'
            || client_mcommands[index].params[j].type == 'l') {
          sprintf(buffer, " %i", defaults->asInt(index, j));
        } else if (client_mcommands[index].params[j].type == 's') {
          sprintf(sparam, "%s", defaults->asString(index, j));
          for (int ic = 0; sparam[ic] != '\0'; ic++) {
            if (sparam[ic]==' ') sparam[ic] = '\a';
          }
          sprintf(buffer, " %s", sparam);
        } else {
          sprintf(buffer, " %f", defaults->asDouble(index, j));
        }
        strcat(request, buffer);
      }
    } else {
      index = SIndex(NCommandList->currentItem()->text());

      // Check to see if this command requires a confirm
      if (client_scommands[index].group & CONFIRM) {
        sprintf(buffer, "The command %s requires confirmation.\n"
            "Are you sure you want to send this command?",
            NCommandList->currentItem()->text().toStdString().c_str());

        if ( QMessageBox::warning(this, "Confirm Command", tr(buffer),
              QMessageBox::Yes, QMessageBox::Escape | QMessageBox::No |
              QMessageBox::Default) == QMessageBox::No ) {
          WriteCmd(NLog, request);
          WriteErr(NLog, 11);
          return;
        }
      }
    }

    //trigger the animation
    dir=1;
    framenum=1;

    /* NetCmd inhibited */
    if (stop_comms)
      return;

    /* Take the conn */
    /* note: loop takes place in TakeConn now. TODO implement limited attempts
    */
    if (!NetCmdTakeConn(!verbose)) { //Never got the conn.
      WriteCmd(NLog, request);
      WriteErr(NLog, 12);
      return;
    }

    TurnOn();

    // Update log file on disk
    WriteCmd(NLog, request);
    WriteLog(request);

    /* Send the command */
    strcat(request, "\r\n");
    NetCmdSend(request);
  } else {
    if (!stop_comms)
      NetCmdSend("::kill::\r\n");
  }
}

void MainForm::ServerDropped() {
  bool ok;
  stop_comms = 1; /* The reset will restart comms */

  QString host=QInputDialog::getText(this,
      "Server Dropped",
      "Connection to server lost.  You may change "
      "servers at this point, if desired, or cancel to close COW",
      QLineEdit::Normal, blastcmd_host, &ok);
  if (!ok) {
    Quit();
    qApp->exit(0);
  } else {
    blastcmd_host=host;
    Quit();
    qApp->exit(1337);
  }
}

void MainForm::ChangeHost() {
  bool ok;
  QString host=QInputDialog::getText(this,
      "Change Host", "What host (or host:port) should cow connect to?", 
      QLineEdit::Normal, blastcmd_host, &ok);
  if(!ok) {
    return;
  } else {
    blastcmd_host=host;
    Quit();
    qApp->exit(1337);
  }
}

//-------------------------------------------------------------
//
// WriteCmd (private): keeps a record of the commands sent
//      with Cow; also records it in a textbox visible to
//      the user
//
//   *dest: the textbox
//   *args: the arguments sent to blastcmd
//
//-------------------------------------------------------------

void MainForm::WriteCmd(QTextEdit *dest, const char *request) {
  FILE *f;
  time_t t;
  char txt[2048];

  t = time(NULL);
  f = fopen(LOGFILE, "a");

  if (f == NULL) {
    QMessageBox::warning(this,"Could not write log","Could not write log. Did you forget to run \"make install\"?");
    fprintf(stderr, "Cow: could not write log file %s.\n", LOGFILE);
    return;
  }

  sprintf(txt, "%s", ctime(&t));
  fprintf(f, "%s", ctime(&t));
  txt[strlen(txt) - 1] = '\0';        // Get rid of the \n character
  dest->insertPlainText("\n"+QString(txt));
  dest->moveCursor(QTextCursor::End);

  sprintf(txt, "  %s\n", request);
  fprintf(f, "  %s\n", request);
  txt[strlen(txt) - 1] = '\0';        // Get rid of the \n character
  dest->insertPlainText("\n"+QString(txt));
  dest->moveCursor(QTextCursor::End);
  fclose(f);
}

void MainForm::WriteLog(const char *request) {
  QDateTime qdt;
  QString LogEntry, Group;
  int i, j;

  Group = client_group_names[GetGroup()];

  qdt = QDateTime::currentDateTime();

  LogEntry = NAboutLabel->text();
  LogEntry += "\n";

  if ((j = MIndex(NCommandList->currentItem()->text()))
      != -1) {
    for (i=0; i < client_mcommands[j].numparams; i++) {
      LogEntry += QString("%1: %2").
        arg(client_mcommands[j].params[i].name, 30).
        arg(dynamic_cast<AbstractCowEntry*>(NParamFields[i])->Text(), -8);
      if (i % 2 == 1)
        LogEntry += "\n";
    }
    LogEntry += "\n";
  }

  LogEntry += "\n";

  LogEntry += &request[3];

  LogEntry += "\nTransmit via ";
  switch (NSendMethod->currentIndex()) {
    case 0:
      LogEntry+="Line of Sight";
      break;
    case 1:
      LogEntry+="TDRSS";
      break;
    case 2:
      LogEntry+="Iridium";
      break;
    case 3:
      LogEntry+="Pilot";
      break;
  }
  if (NSendMethod->currentIndex() != 3) {
    LogEntry+= " through SIP ";
    switch (NSendRoute->currentIndex()) {
      case 0:
        LogEntry+="COMM 1";
        break;
      case 1:
        LogEntry+="COMM 2";
        break;
    }
  }

  LogEntry += "\n";
  LogEntry += QString(
      "-- Time  : %1\n"
      "-- Source: %2  Type: %3  Entry By: %4\n"
      "-- Frame : %5  File: %6  \n")
    .arg(qdt.toString())
    .arg("Cow",-10)
    .arg(Group, -10)
    .arg((getpwuid(getuid()))->pw_name, -10)
    .arg(_dirfile ? _dirfile->NFrames() : -1, -10)
    .arg(_dirfile ? _dirfile->Name() : "");

  LogEntry+="--------------------------------------------------\n\n";

  QString logfilename = LOGFILEDIR +
    qdt.toString("yyyy-MM-dd.hh:mm:ss") + "." +
    "Cow." +
    Group.replace(" ", "_")+ "." +
    QString((getpwuid(getuid()))->pw_name);

  QFile logfile(logfilename);

  if (logfile.open(QFile::WriteOnly)) {
    QTextStream stream(&logfile);
    stream << LogEntry;
    logfile.close();
  }

}

void MainForm::ReadHerdFile(const QString &herd_file_name)
{
  QFile file(herd_file_name);

  QString herd;

  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return;
  }

  while (!file.atEnd()) {
    QByteArray line = file.readLine();
    if (line.contains('#')) {
      line.truncate(line.indexOf('#'));
    }
    line = line.trimmed();
    if (line.isEmpty()) {
      continue;
    }

    if (line.startsWith('[')) {
      if (line.endsWith(']')) {
        line.remove(0,1);
        line.chop(1);
        herd = QString(line);
        ListOfHerds.append(herd);
      }
    } else {
      HerdHash[herd].append(QString(line));
    }
  }
}


//-------------------------------------------------------------
//
// WriteErr: for when the user cancels a command or blastcmd
//      reports that it didn't go through
//
//   *dest: textbox to write into
//   retstatus: what went wrong
//
//-------------------------------------------------------------

void MainForm::WriteErr(QTextEdit *dest, const char *message, int retstatus) {
  QString txt;

  switch (retstatus) {
    case 99:
      txt = "  COMMAND NOT SENT:  Cancelled by user.\n";
      break;
    case 0:
      txt = "  Command successfully sent.\n";
      break;
    case 1:
      txt = "  COMMAND NOT SENT:  Improper syntax. (Have you compiled "
        "with an up-to-date verison of commands.h?)\n\n";
      break;
    case 2:
      txt = "  COMMAND NOT SENT:  Unable to open serial port.\n\n";
      break;
    case 3:
      txt = "  COMMAND NOT SENT:  Parameter out of range. (Have you "
        "compiled with an up-to-date version of commands.h?)\n";
      break;
    case 4:
      txt = "  COMMAND NOT SENT:  GSE operator disabled science from "
        "sending commands.\n";
      break;
    case 5:
      txt = "  COMMAND NOT SENT:  Routing address does not match the "
        "selected link.\n";
      break;
    case 6:
      txt = "  COMMAND NOT SENT:  The link selected was not "
        "enabled.\n";
      break;
    case 7:
      txt = "  COMMAND NOT SENT:  Unknown error from ground support "
        "computer (0x0d).\n";
      break;
    case 8:
    case 9:
      txt = "  COMMAND POSSIBLY NOT SENT:  Received a garbage "
        "acknowledgement packet.\n";
      break;
    case 10:
      txt = "  COMMAND NOT SENT: Cow error: Parameter out of range.\n";
      break;
    case 11:
      txt = "  COMMAND NOT SENT: Command not confirmed by user.\n";
      break;
    case 12:
      txt = "  COMMAND NOT SENT: Unable to take the conn.\n";
      break;
    case 13:
      txt = "  COMMAND POSSIBLY NOT SENT: Timeout waiting for ack from GSE.\n";
      break;
    case 17:
      txt = "  COMMAND NOT SENT: Parameter validation failed.\n";
      if (message) {
        txt += "  Error: ";
        txt += message;
        txt += "\n";
      }
      break;
    case 18:
      txt = "  COMMAND NOT SENT: Error forwarding command.\n";
      break;
    case 19:
      txt = "  COMMAND NOT SENT: Protocol error.\n";
      break;
    case 20:
      txt = "  COMMAND NOT SENT: Link/route pair disabled by command server.\n";
      break;
    default:
      txt = "  COMMAND POSSIBLY NOT SENT: Unrecognised response from server.\n";
      break;
  }

  QFile f(LOGFILE);
  if (f.open(QFile::Append | QFile::WriteOnly)) {
    QTextStream stream( &f );
    stream << txt;
    f.close();
  } else {
    fprintf(stderr, "Cow: could not write log file %s.\n", LOGFILE);
    return;
  }

  dest->insertPlainText("\n"+QString(txt));
  dest->moveCursor(QTextCursor::End);
}

void MainForm::WriteErr(QTextEdit *dest, int retstatus) {
  WriteErr(dest, NULL, retstatus);
}




//-------------------------------------------------------------
//
// ReadLog (private): read in the log when Cow starts
//
//   *dest: the textbox to write to
//
//-------------------------------------------------------------

void MainForm::ReadLog(QTextEdit *dest) {
  FILE *f;
  char txt[255];

  f = fopen(LOGFILE, "r");
  dest->setText(" ");

  if (f == NULL) {
    printf("Cow:  could not read log file %s.\n", LOGFILE);
    return;
  }

  while(fgets(txt, 255, f) != NULL) {
    txt[strlen(txt) - 1] = '\0'; // Get rid of the \n character from each line
    dest->insertPlainText("\n"+QString(txt));
  }
  dest->insertPlainText("\n");
  dest->moveCursor(QTextCursor::End);
}

void MainForm::keyPressEvent(QKeyEvent *ev)
{
  if (SEND_KEY) {
    NSendButton->animateClick(100);
  } else if(dynamic_cast<AbstractCowEntry*>(focusWidget())) {
    QMainWindow::keyPressEvent(ev);
  } else {
    NOmniBox->keyPressEvent(ev);
  }
}

bool operator<(const MainForm::OmniPair& a,const MainForm::OmniPair&b)
{
  return a.name<b.name;
}

bool operator==(const MainForm::OmniPair& a,const MainForm::OmniPair&b)
{
  return a.name==b.name;
}

void MainForm::PopulateOmnibox()
{
  //    NOmniBox->clear();
  OmniList.clear();

  for(int h=0;h<client_n_groups;h++)
  {
    int indexes[client_n_scommands + client_n_mcommands],max;

    max = GroupSIndexes(h, indexes);
    max += GroupMIndexes(h, &indexes[max]);

    for (int i = 0; i < max; i++) {
      OmniPair pair;
      pair.group=h;
      if (indexes[i] >= client_n_scommands) {
        pair.name=client_mcommands[indexes[i] - client_n_scommands].name;
      } else {
        pair.name=client_scommands[indexes[i]].name;
      }
      //if(!OmniList.contains(pair)) {
      OmniList.push_back(pair);
      //}
    }
  }

  qSort(OmniList);

  QStringList sl;
  for(int i=0;i<OmniList.size();i++)
  {
    sl.push_back(OmniList[i].name+" ");
  }
  sl.removeDuplicates();

  if(NOmniBox->completer()) {
    delete NOmniBox->completer();
  }

  QCompleter *qcomp = new QCompleter(sl);
  //can conceivably use SIGNAL(highlighted) instead, but I think this behaviour is better
  connect(qcomp, SIGNAL(activated(const QString &)), this, SLOT(nOmniBox_completerActivated(const QString &)));
  NOmniBox->setCompleter(qcomp);
}

//-------------------------------------------------------------
//
// MainForm: constructor
//
//-------------------------------------------------------------

MainForm::MainForm(const char *cf, const QString &herdfile, int link, QWidget* parent,  const char* name,
    Qt::WindowFlags fl) : QMainWindow( parent, fl )
{
  setObjectName(name?QString(name):"Cow");
  int i;
  char tmp[SIZE_NAME + SIZE_ABOUT + SIZE_PARNAME];
  QSize tempsize;
  QPoint point;
  int w1, w2, w3, h1, h2, h3;

  QSettings settings("University of Toronto","cow");

  restoreGeometry(settings.value("geometry").toByteArray());

  ReadHerdFile(herdfile);

  _dirfile = NULL;

  centralWidget = new QWidget();
  theHLayout = new QHBoxLayout;
  theHLayout->addLayout(theVLayout = new QVBoxLayout());

  curfile = cf;
  lastmcmd = -1;
  sending = 0;
  verbose = 0;
  pong = 1;

  Images[0] = new QPixmap(":/icons/lightning00.png");
  Images[1] = new QPixmap(":/icons/lightning03.png");
  Images[2] = new QPixmap(":/icons/lightning06.png");
  Images[3] = new QPixmap(":/icons/lightning09.png");
  Images[4] = new QPixmap(":/icons/lightning12.png");
  Images[5] = new QPixmap(":/icons/lightning15.png");

  QString username = qgetenv("USER");
  if (username.isEmpty())
    username = qgetenv("USERNAME");

  // if not SJB, also work when COW_BUTT environment variable is set
  bool cowbutt = !qgetenv("COW_BUTT").isNull();

  if (username == "sjb" || cowbutt) {
     Images[0] = new QPixmap(":/icons/cow0.png");
     Images[1] = new QPixmap(":/icons/cow1.png");
     Images[2] = new QPixmap(":/icons/cow2.png");
     Images[3] = new QPixmap(":/icons/cow3.png");
     Images[4] = new QPixmap(":/icons/cow4.png");
     Images[5] = new QPixmap(":/icons/cow5.png");
  }

  framenum = 0;
  numframes = 4;
  dir = 1;

  sprintf(tmp, "Command Operations Window (COW, revision." COW_SVN_REVISION
      "); Host: %s; Cmd List: %s",
      blastcmd_host.toStdString().c_str(), client_command_list_serial);

  setWindowTitle(tmp);

  NOmniBox = new CowOmniBox(this->centralWidget);
  NOmniBox->setObjectName("NFilter");
  NOmniBox->adjustSize();
  //NOmniBox->setPlaceholderText("Awesome Bar");
  connect(NOmniBox,SIGNAL(textEdited(QString)),this,SLOT(nOmniBox_textEdited(QString)));
  //connect(NOmniBox,SIGNAL(textChanged(QString)),this,SLOT(testTextChanged(QString)));       //for debugging
  //connect(NOmniBox,SIGNAL(cursorPositionChanged(int,int)),this,SLOT(testCursorPositionChanged(int,int)));
  theHLayout->addWidget(NOmniBox);

  // Lay everything out.  Everything is very carefully positioned -- there are
  // no automatic spacers because with the dynamic properties of the program
  // (things popping in and out of existence as different commands are
  // chosen) things would mess up.  So the code ain't too pretty . . .
  TabWidget = new QTabWidget(centralWidget);
  tab1 = new QWidget();

  NGroupsBox = new QGroupBox(tab1);
  NGroupsBox->setObjectName("NGroupsBox");
  NGroupsBox->setFlat(true);
  //theVLayout->addWidget(NGroupsBox);

  NGroupsLayout = new QGridLayout(NGroupsBox);
  NGroupsLayout->setAlignment(Qt::AlignTop);
  NGroupsLayout->setSpacing(1);
  NGroupsLayout->setMargin(5);

  NGroups = new QRadioButton*[client_n_groups];
  for (i = 0; i < client_n_groups; i++) {
    NGroups[i] = new QRadioButton(NGroupsBox);
    //connect(NGroups[i],SIGNAL(clicked()),NOmniBox,SLOT(clear()));
    connect(NGroups[i],SIGNAL(toggled(bool)),this,SLOT(ChangeCommandList(bool)));
    NGroups[i]->setObjectName("QGroup");
    NGroups[i]->setText(tr(client_group_names[i]));
    tempsize = NGroups[i]->sizeHint();
    NGroupsLayout->addWidget(NGroups[i], int(i/3), (i%3));
  }

  TabWidget->addTab(tab1,"Default");

  tab2 = new QWidget();
  HerdGroupBox = new QGroupBox(tab2);
  HerdGroupBox->setObjectName("NGroupsBox");
  HerdGroupBox->setFlat(true);

  HerdGroupsLayout = new QGridLayout(HerdGroupBox);
  HerdGroupsLayout->setAlignment(Qt::AlignTop);
  HerdGroupsLayout->setSpacing(1);
  HerdGroupsLayout->setMargin(5);

  HerdGroups = new QRadioButton*[ListOfHerds.length()];
  for (i = 0; i < ListOfHerds.length(); i++) {
    HerdGroups[i] = new QRadioButton(HerdGroupBox);
    connect(HerdGroups[i],SIGNAL(toggled(bool)),this,SLOT(ChangeCommandList(bool)));
    //HerdGroups[i]->setObjectName("QGroup");
    HerdGroups[i]->setText(ListOfHerds.at(i));
    HerdGroupsLayout->addWidget(HerdGroups[i], int(i/3), (i%3));
  }

  TabWidget->addTab(tab2, "Custom");

  theVLayout->addWidget(TabWidget);
  connect(TabWidget, SIGNAL(currentChanged(int)), this, SLOT(ChangeCommandList()));

  NTopFrame = new QFrame(this->centralWidget);
  NTopFrame->setObjectName("NTopFrame");
  NTopFrame->setFrameShape(QFrame::Box);
  NTopFrame->setFrameShadow(QFrame::Sunken);
  theVLayout->addWidget(NTopFrame);


  NCommandList = new QListWidget(this->centralWidget);
  NCommandList->setSelectionMode(QAbstractItemView::SingleSelection);
  NCommandList->setObjectName("NCommandList");
  NCommandList->adjustSize();
  NCommandList->setGeometry(PADDING, PADDING+120, NCommandList->width(), 0);
  connect(NCommandList, SIGNAL(currentRowChanged(int)), this, SLOT(ChooseCommand()));
  connect(NCommandList, SIGNAL(clicked(QModelIndex)), this, SLOT(ChooseCommand()));
  theHLayout->addWidget(NCommandList);

  w1 = 0;

  strcpy(tmp, LongestParam());
  for (i = 0; i < MAX_N_PARAMS; i++) {
    NParamLabels[i] = new QLabel(NTopFrame);
    NParamLabels[i]->setObjectName("NParamLabel");
    NParamLabels[i]->setText(tr(tmp));
    NParamLabels[i]->setAlignment(Qt::AlignHCenter);
    NParamLabels[i]->adjustSize();
    w2 = NParamLabels[i]->width();

    NParamFields[i] = new CowDoubleEntry(NTopFrame, "NParamLabels");
    connect(dynamic_cast<CowDoubleEntry*>(NParamFields[i]),
        SIGNAL(valueEdited()),this,SLOT(OmniSync()));

    NParamFields[i]->setFixedWidth(w2/2);
    NParamFields[i]->adjustSize();

    w3 = NParamFields[i]->width();
    h2 = NParamLabels[i]->height();
    h3 = NParamFields[i]->height();

    point.setX(w1 + 2 * PADDING + (i % 2) * (w2 + w3 + PADDING));
    NParamLabels[i]->setGeometry(point.x(), 0, w2, h2);

    point.setX(w1 + PADDING + (i % 2) * (w2 + w3 + PADDING) + w2);
    NParamFields[i]->setGeometry(point.x(), 0, w3, h3);
    if (i>0) {
        setTabOrder(NParamFields[i-1], NParamFields[i]);
    }
  }

  memset(tmp, 'B', SIZE_ABOUT);
  tmp[SIZE_ABOUT] = '\0';

  NAboutLabel = new QLabel(NTopFrame);
  NAboutLabel->setObjectName("NAboutLabel");
  NAboutLabel->setFrameShape(QFrame::Box);
  NAboutLabel->setFrameShadow(QFrame::Plain);
  NAboutLabel->setText(tr(tmp));
  NAboutLabel->setWordWrap(1);
  NAboutLabel->setAlignment(Qt::AlignLeft);
  NAboutLabel->setGeometry(0, 0, 2 * (w2 + w3 + PADDING) + SPACING * 4, 0);
  tempsize = NAboutLabel->sizeHint();
  NAboutLabel->setGeometry(PADDING , PADDING*2, 2 * (w2 + w3 + PADDING)
      + SPACING * 4, tempsize.height());

  h1 = NAboutLabel->height()+PADDING;
  for (i = 0; i < MAX_N_PARAMS; i++) {
    w2 = NParamLabels[i]->width();
    w3 = NParamFields[i]->width();
    h2 = NParamLabels[i]->height();
    h3 = NParamFields[i]->height();

    point.setX(w1 + 2 * PADDING + (i%2) * (w2 + w3 + PADDING + SPACING * 4));
    point.setY(h1 + 2 * PADDING + int(i/2) * (h3 + SPACING) + int((h3 - h2)/2));
    NParamLabels[i]->setGeometry(point.x(), point.y(), w2, h2);

    point.setX(w1 + 3 * PADDING + i * (w2 + w3 + PADDING + SPACING * 4) + w2 -
        2 * int(i/2) * (w2 + w3 + PADDING + SPACING * 4));
    point.setY(h1 + 2 * PADDING + int(i/2) * (h3 + SPACING));
    NParamFields[i]->setGeometry(point.x(), point.y(), w3, h3);

    NParamFields[i]->hide();
    NParamLabels[i]->hide();
  }

  NCurFile = new QLineEdit(NTopFrame);
  NCurFile->setObjectName("NCurFile");
  NCurFile->setText(tr(curfile.toLatin1()));
  NCurFile->adjustSize();
  QCompleter* completer=new QCompleter;
  completer->setModel(new QDirModel(completer));
  NCurFile->setCompleter(completer);

  NHost = new QPushButton(NTopFrame);
  NHost->setObjectName("NHost");
  NHost->setText(blastcmd_host);
  connect(NHost,SIGNAL(clicked()),this,SLOT(ChangeHost()));

  NSendMethod = new QComboBox(NTopFrame);
  NSendMethod->setObjectName("NSendMethod");
  NSendMethod->addItem(tr("LOS"));
  NSendMethod->addItem(tr("TDRSS"));
  NSendMethod->addItem(tr("Iridum"));
  NSendMethod->addItem(tr("Pilot"));
  NSendMethod->adjustSize();
  NSendMethod->setCurrentIndex(link);

  NSendRoute = new QComboBox(NTopFrame);
  NSendRoute->setObjectName("NSendRoute");
  NSendRoute->addItem(tr("COMM 1"));
  NSendRoute->addItem(tr("COMM 2"));
  NSendRoute->adjustSize();

  NVerbose = new QCheckBox(NTopFrame);
  NVerbose->setObjectName("NVerbose");
  NVerbose->setText(tr("Verbose"));
  NVerbose->setChecked(false);
  NVerbose->adjustSize();

  NCurFile->setGeometry(
      PADDING,
      PADDING + 2 * PADDING + h1 + (int((2 + MAX_N_PARAMS) / 2)) * (h3 + SPACING) - NCurFile->height(),
      NCurFile->width()*1.5,
      NCurFile->height());

  NHost->setGeometry(
      3*PADDING+NCurFile->width(),
      PADDING + 2 * PADDING + h1 + (int((2 + MAX_N_PARAMS) / 2)) * (h3 + SPACING) - NCurFile->height(),
      NCurFile->width()*(1.0/1.5),
      NCurFile->height());

  NSendMethod->setGeometry(
      4*PADDING+NCurFile->width()+NHost->width(),
      NCurFile->y(),
      NSendMethod->width(),
      NCurFile->height());

  NSendRoute->setGeometry(
      5*PADDING+NCurFile->width()+NHost->width()+NSendMethod->width(),
      NCurFile->y(),
      NSendRoute->width(),
      NCurFile->height());

  NVerbose->setGeometry(
      6*PADDING+NCurFile->width()+NHost->width()+NSendMethod->width()+NSendRoute->width(),
      NCurFile->y(),
      NVerbose->width(),
      NCurFile->height());

  NTopFrame->adjustSize();

  NGroupsBox->adjustSize();
  NGroupsBox->setGeometry(0,0,NTopFrame->width(), NGroupsBox->height());

  HerdGroupBox->adjustSize();
  HerdGroupBox->setGeometry(0,0,NTopFrame->width(), HerdGroupBox->height());

  int xleft = 2 * PADDING + NCommandList->width();
  int ytop = PADDING*2+25;
  int width = NTopFrame->width();
  int height = TabWidget->tabBar()->height()+NGroupsBox->height()+2*PADDING;

  TabWidget->adjustSize();
  TabWidget->setGeometry(xleft, ytop, width, height);

  NTopFrame->adjustSize();
  ytop = ytop + height;
  width = NTopFrame->width();
  height = NTopFrame->height();
  NTopFrame->setGeometry(xleft, ytop, width, height);

  NBotFrame = new QFrame(this->centralWidget);
  NBotFrame->setObjectName("NBotFrame");
  NBotFrame->setFrameShape(QFrame::Box);
  NBotFrame->setFrameShadow(QFrame::Sunken);
  theVLayout->addWidget(NBotFrame);

  NSendButton = new QPushButton(NBotFrame);
  NSendButton->setObjectName("NSendButton");
  NSendButton->setText(tr("Send (" SEND_KEY_NAME ")"));
  NSendButton->adjustSize();
  NSendButton->setDisabled(true);

  NWaitImage = new QLabel(NBotFrame);
  NWaitImage->setText("NWaitImage");
  NWaitImage->setScaledContents(false);
  NWaitImage->setPixmap(*Images[0]);
  NWaitImage->adjustSize();
  NWaitImage->setGeometry(
      NTopFrame->width() - NWaitImage->width() - PADDING,
      PADDING,
      NSendButton->width(),
      NWaitImage->height());


  NSendButton->setGeometry(
      NTopFrame->width() - NWaitImage->width() - PADDING,
      PADDING+NWaitImage->height(),
      NSendButton->width(),
      NSendButton->height());

  NLog = new QTextEdit(NBotFrame);
  NLog->setObjectName("NLog");
  NLog->setReadOnly(true);
  NLog->setGeometry(
      PADDING,
      PADDING,
      NTopFrame->width() - NWaitImage->width() - 3 * PADDING,
      PADDING + NWaitImage->height()+NSendButton->height());

  ReadLog(NLog);

  NBotFrame->adjustSize();

  ytop = ytop + NTopFrame->height();

  NBotFrame->setGeometry(xleft, ytop, NTopFrame->width(),
      NBotFrame->height());

  NOmniBox->setGeometry(PADDING,PADDING,NCommandList->width()+PADDING+NTopFrame->width(),25);

  NCommandList->setGeometry(PADDING, PADDING+NOmniBox->height()+PADDING,
                            NCommandList->width(), ytop+NBotFrame->height()-25-2*PADDING);

  timer = new QTimer(this);
  timer->setObjectName("image_timer");
  timer->start(WAIT_TIME);

  ChangeCommandList();
  ChooseCommand();

  setCentralWidget(centralWidget);

  theStatusBar = statusBar();
  theStatusBar->setSizeGripEnabled(false);

  ConnBanner = new QLabel(theStatusBar);
  ConnBanner->setText(NetCmdBanner());
  theStatusBar->addWidget(ConnBanner);

  w1 = NCommandList->width() + NGroupsBox->width() + 2 * PADDING + SPACING;
  h1 = NCommandList->height() + theStatusBar->height() + NOmniBox->height();

  setMinimumSize(w1, h1);
  setMaximumSize(w1, h1);

  ping_timer = new QTimer(this);
  ping_timer->setObjectName("ping_timer");
  ping_timer->start(PING_TIME);

  PopulateOmnibox();

  connect(NSendButton, SIGNAL(clicked()), this, SLOT(SendCommand()));
  connect(timer, SIGNAL(timeout()), this, SLOT(Tick()));
  connect(ping_timer, SIGNAL(timeout()), this, SLOT(Ping()));

  NGroups[0]->setChecked(true);

  NOmniBox->setFocus();
}

MainForm::~MainForm()
{
  Quit();
  delete _dirfile;
  _dirfile = NULL;

  for (int i = 0; i < 6; i++)
    delete Images[i];

  delete theVLayout;
  delete theHLayout;
  delete centralWidget;
  delete theStatusBar;
}


//***************************************************************************
//****     Main()
//***************************************************************************

// Who ever though raising SIGABRT when QApplication couldn't connect to the
// display would be a good idea?
void handle_sigabrt(int sig)
{
  Q_UNUSED(sig);
  std::cerr << "The Cow says: \"I can't connect to your DISPLAY." << std::endl;
  std::cerr << "                Have you tried 'ssh -YC'?\"" << std::endl;
  exit(1);
}

void USAGE() {
    fprintf(stderr, "USAGE:\n"
            "cow <command host>\n"
            "or\n"
            "cow <options>\n"
            "\n"
            "Options:\n"
            "-h     print this message\n"
            "-c <command host>\n"
            "-H <herd file>\n");
    exit(1);
}

Defaults *defaults;
int main(int argc, char* argv[]) {

  // Catch QApplication's SIGABRT
  signal(SIGABRT, &handle_sigabrt);
  QApplication app(argc, argv);

  // Clear signal handler
  signal(SIGABRT, SIG_DFL);
  QApplication::setOrganizationName("University of Toronto");
  QApplication::setApplicationName("cow");

  /* Host determination */
  if (argc == 2) {
      if (argv[1][0] == '-') {
          USAGE();
      } else {
          blastcmd_host = QString(argv[1]);
      };
  } else {
      QString argv_i;
      for (int i = 1; i<argc; i++) {
          argv_i = QString(argv[i]);
          if (argv_i == "-h") {
              USAGE();
          } else if (argv_i == "-c") {
              i++;
              if (i>=argc) USAGE();
              blastcmd_host = QString(argv[i]);
          } else if (argv_i == "-H") {
              i++;
              if (i>=argc) USAGE();
              herdfile = QString(argv[i]);
          } else {
              USAGE();
          }
      }
  }

  QSettings settings("University of Toronto","cow");

  /* Link Determination */
  int link = settings.value("link", 0).toInt();

  if (blastcmd_host.isEmpty()) {
      blastcmd_host = settings.value("blastcmd_host",QString("widow\0")).toString();
  }

  if (herdfile.isEmpty()) {
      herdfile = settings.value("blastcmd_herdfile",QString("widow\0")).toString();
  }

  if (!herdfile.endsWith(".herd")) {
      herdfile = herdfile + ".herd";
  }

  if (!QFile::exists(herdfile)) {
      herdfile = QString(HERDFILEDIR) + herdfile;
  }

  QFileInfo herd_fileinfo(herdfile);

  herdfile = herd_fileinfo.canonicalFilePath();
    printf("%s\n", herdfile.toLatin1().constData());
  if (!QFile::exists(herdfile)) {
      printf("warning: herdfile (%s) does not exist\n", herdfile.toLatin1().constData());
  }

  settings.setValue("blastcmd_host",QString(blastcmd_host));
  settings.setValue("blastcmd_herdfile",QString(herdfile));

  int retCode=1337;
  while(retCode==1337) {
    int n;
    /* Client negotiation */
    stop_comms = 0;
    missed_pongs = 0;
    while ((n = NetCmdConnect(blastcmd_host.toStdString().c_str(), 1, 0)) < 0) {
      if (n == -16) { /* unauthorised */
        bool ok;
        blastcmd_host=QInputDialog::getText(0, "Unauthorised",
            "You are not authorised to use the blastcmd daemon on " +
            QString(blastcmd_host) + ". Try another hostname.",
            QLineEdit::Normal,QString(blastcmd_host),&ok).toLatin1();
        if (!ok)
          exit(16);
      } else {
        //retry if connection refused (HACK! fixes bug in blastcmd
        //authentication)
        sleep(1);
        printf("Trying to connect one more time\n");
        if (NetCmdConnect(blastcmd_host.toStdString().c_str(), 1, 0) < 0) {
          bool ok;
          blastcmd_host=QInputDialog::getText(0, "Bad host",
              "Could not connect to " + QString(blastcmd_host) +
              " or blastcmd daemon. Try another hostname.",
              QLineEdit::Normal,QString(blastcmd_host),&ok).toLatin1();
          if (!ok)
            exit(16);
        }
      }
    }
    if (NetCmdGetCmdList())
      return -1;
    if (NetCmdGetGroupNames())
      return -1;


    defaults = new Defaults();

    QSettings settings;
    QString curfile=settings.value("curfile",QString("")).toString();
    if(curfile.isEmpty()) {
      curfile= "/data/etc/mole.lnk"; //QFileDialog::getExistingDirectory(0,"Choose a curdir");
    }


    MainForm moo(curfile.toStdString().c_str(), herdfile, link, 0, "moo", 0);
    moo.show();
    retCode= app.exec();
    NetCmdDrop();
  }
  return retCode;
}

// vim: ts=2 sw=2 et
