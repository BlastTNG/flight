/* narsil: GUI commanding front-end
 *
 * This software is copyright (C) 2002-2004 University of Toronto
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
// *  Later poked at a bit by D.V.Wiebe              *
// *  further desecrated by cbn                      *
// ***************************************************

#include <qapplication.h>
#include <qbuttongroup.h>
#include <qframe.h>
#include <qlabel.h>
#include <qlineedit.h>
#include <qlistbox.h>
#include <qpushbutton.h>
#include <qradiobutton.h>
#include <qlayout.h>
#include <qvariant.h>
#include <qtooltip.h>
#include <qwhatsthis.h>
#include <qspinbox.h>
#include <qvalidator.h>
#include <qpixmap.h>
#include <qcheckbox.h>
#include <qcombobox.h>
#include <qmessagebox.h>
#include <qmultilineedit.h>
#include <qtimer.h>
#include <qfile.h>
#include <qdatetime.h>

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

#define FIXEDFONT "courier"

#include "narsil.h"
#include "kstfile.h"
#include "doubleentry.h"

#include <iostream>

#define PADDING 3
#define SPACING 3

#define DEF_CURFILE CUR_DIR "/defile.cur"
#define LOGFILE DATA_ETC_NARSIL_DIR "/log.txt"
#define LOGFILEDIR DATA_ETC_NARSIL_DIR "/log/"

#define BLASTCMDFILE BLAST_CMD
#define DATA_DIR DATA_ETC_NARSIL_DIR

#ifndef ELOG_HOST
#define ELOG_HOST "elog.blast"
#endif

/* Defaults class holds the default parameter values */
Defaults::Defaults()
{
  int i, j;
  int fp;
  int n_read = 0;

  /* Read in previous default values */
  if ((fp = open(DATA_DIR "/prev_status", O_RDONLY)) >= 0) {
    n_read = read(fp, &rdefaults, sizeof(double) * N_MCOMMANDS * MAX_N_PARAMS);
    n_read += read(fp, &idefaults, sizeof(int) * N_MCOMMANDS * MAX_N_PARAMS);
    close(fp);
  }

  if (n_read != sizeof(rdefaults) + sizeof(idefaults))
    for (i = 0; i < N_MCOMMANDS; i++)
      for (j = 0; j < MAX_N_PARAMS; j++)
        rdefaults[i][j] = idefaults[i][j] = 0;
}

//-------------------------------------------------------------
//
// Defaults::Save: Narsil remembers any values entered for the
//      next time it starts up
//
//-------------------------------------------------------------

void Defaults::Save() {
  int fp;

  /* Write file with defaults */
  fp = open(DATA_DIR "/prev_status", O_WRONLY|O_CREAT|O_TRUNC, 0666);
  if (fp < 0)
    printf("Warning: could not open prev_status file\n");
  else {
    write(fp, &rdefaults, sizeof(rdefaults));
    write(fp, &idefaults, sizeof(idefaults));
    close(fp);
  }
}

void Defaults::Set(int i, int j, QString text)
{
  idefaults[i][j] = text.toInt();
  rdefaults[i][j] = text.toDouble();
}

int Defaults::asInt(int i, int j) { return idefaults[i][j]; }
double Defaults::asDouble(int i, int j) { return rdefaults[i][j]; }

//|||***************************************************************************
//|||****
//|||****
//|||****     CLASS MainForm -- main control class
//|||****
//|||****
//|||****_______________________________________________________________________
//|||***************************************************************************


//-------------------------------------------------------------
// GetGroup (private): checks the radio group of buttons to see
//      which group is checked
//
//   Returns: index of the selected group
//-------------------------------------------------------------

int MainForm::GetGroup() {
  int i;

  for (i = 0; i < N_GROUPS; i++)
    if (NGroups[i]->isChecked())
      return i;
}

int TheSort(const void* a, const void* b)
{
  const char *za = (*(int*)a >= N_SCOMMANDS) ? mcommands[*(int*)a -
    N_SCOMMANDS].name : scommands[*(int*)a].name;
  const char *zb = (*(int*)b >= N_SCOMMANDS) ? mcommands[*(int*)b -
    N_SCOMMANDS].name : scommands[*(int*)b].name;
  return strcmp(za, zb);
}

//-------------------------------------------------------------
// ChangeCommandList (slot): when a new group is selected,
//      the list of commands must be cleared and the new
//      group's commands added.  Triggered when a group button
//      is selected.
//-------------------------------------------------------------

void MainForm::ChangeCommandList() {
  int indexes[N_SCOMMANDS + N_MCOMMANDS];
  int i;
  int max;

  NCommandList->clearSelection();
  NCommandList->clearFocus();
  NCommandList->clear();

  max = GroupSIndexes(GetGroup(), indexes);
  max += GroupMIndexes(GetGroup(), &indexes[max]);
  qsort(indexes, max, sizeof(int), &TheSort);

  for (i = 0; i < max; i++)
    if (indexes[i] >= N_SCOMMANDS)
      NCommandList->insertItem(mcommands[indexes[i] - N_SCOMMANDS].name);
    else
      NCommandList->insertItem(scommands[indexes[i]].name);

  ChooseCommand();
}

//-------------------------------------------------------------
// ChooseCommand (slot): triggered when a command is selected
//      from the list. Changes labels and shows spin-boxes
//      as appropiate for the command.
//-------------------------------------------------------------

void MainForm::ChooseCommand() {
  int i, index;
  double indata;

  // Remember parameter values
  if (lastmcmd != -1) {
    for (i = 0; i < mcommands[lastmcmd].numparams; i++)
      NParamFields[i]->RecordDefaults();

    defaults->Save();
  }

  // It can happen that this function be called with nothing selected
  if (NCommandList->currentItem() == -1 || !NCommandList->hasFocus()) {
    NSendButton->setDisabled(true);
    lastmcmd = -1;
    NAboutLabel->setText(tr("No command selected."));
    for (i = 0; i < MAX_N_PARAMS; i++) {
      NParamLabels[i]->hide();
      NParamFields[i]->hide();
    }
  } else {
    NSendButton->setEnabled(true);
    if ((index = SIndex(NCommandList->text(NCommandList->currentItem())))
        != -1) {
      // Set up for a single command
      NAboutLabel->setText(scommands[index].about);
      lastmcmd = -1;
      for (i = 0; i < MAX_N_PARAMS; i++) {
        NParamLabels[i]->hide();
        NParamFields[i]->hide();
      }
    } else if ((index = MIndex(NCommandList->text(NCommandList->currentItem())))
        != -1) {
      // Set up for a multi command -- show the parameter spin boxes
      NAboutLabel->setText(mcommands[index].about);
      lastmcmd = index;

      bool IsData = DataSource->update();

      for (i = 0; i < MAX_N_PARAMS; i++) {
        if (i < mcommands[index].numparams) {
          NParamLabels[i]->setText(mcommands[index].params[i].name);
          NParamLabels[i]->show();
          NParamFields[i]->show();
          NParamFields[i]->SetParentField(index, i);
          NParamFields[i]->SetType(mcommands[index].params[i].type);
          if (IsData) {
            if (DataSource->readField(&indata,
                                      mcommands[index].params[i].field,
                                      DataSource->numFrames() - 2, -1) == 0)
              NParamFields[i]->SetDefaultValue(index, i);
            else
              NParamFields[i]->SetValue(indata);
          } else 
            NParamFields[i]->SetDefaultValue(index, i);
        } else {
          NParamLabels[i]->hide();
          NParamFields[i]->hide();
        }
      }
    }
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

  for (i = 0; i < N_SCOMMANDS; i++)
    if (scommands[i].group & (1 << group))
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

  for (i = 0; i < N_MCOMMANDS; i++)
    if (mcommands[i].group & (1 << group))
      indexes[num++] = i + N_SCOMMANDS;

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

  for (i = 0; i < N_SCOMMANDS; i++) {
    if (strcmp(scommands[i].name, cmd) == 0)
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

  for (i = 0; i < N_MCOMMANDS; i++) {
    if (strcmp(mcommands[i].name, cmd) == 0)
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
  int i, j;
  int len = 0;
  static char lp[120];

  for (i = 0; i < N_MCOMMANDS; i++) {
    for (j = 0; j < mcommands[i].numparams; j++) {
      if (strlen(mcommands[i].params[j].name) > len) {
        len = strlen(mcommands[i].params[j].name);
        strcpy(lp, mcommands[i].params[j].name);
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
  int i;

  // Remember parameter values
  if (lastmcmd != -1) {
    for (i = 0; i < mcommands[lastmcmd].numparams; i++)
      NParamFields[i]->RecordDefaults();

    defaults->Save();
  }

  // Release control of program; give it back to Main()
  accept();
}


//-------------------------------------------------------------
//
// ChangeImage (slot): triggered by timer.  Animate the cool
//      picture of Strider; wait for BlastCmd to finish running
//      and return result of send.
//
//-------------------------------------------------------------

void MainForm::ChangeImage() {
  int returnstatus;

  if (sending) {
    if (framenum == 0 && dir == -1)
      dir = 1;
    framenum += dir;
    if (framenum == numframes - 1)
      dir = -1;
    if (framenum == 1)
      dir = 1;
  } else {
    if (framenum != 0)
      framenum = 0;
  }

  NWaitImage->setPixmap(*Images[framenum]);

  if (sending) {
    if (waitpid(sendingpid, &returnstatus, WNOHANG)) {
      if (returnstatus == 9)
        returnstatus = -1;
      else
        returnstatus = WEXITSTATUS(returnstatus);
      WriteErr(NLog, returnstatus);
      TurnOff(timer);
    }
  }
}


//-------------------------------------------------------------
//
// TurnOn (private): called when the user tries to send a
//      command.  Most user input is suspended while the
//      command is being send.
//
//   *t: timer to use for the animation
//
//-------------------------------------------------------------

void MainForm::TurnOn(QTimer *t) {
  int i;

  sending = true;

  NGroupsBox->setDisabled(true);
  NCurFileButton->setDisabled(true);
  NCurFile->setDisabled(true);
  NCommandList->setDisabled(true);
  NSettingsButton->setDisabled(true);
  NAboutLabel->setDisabled(true);
  for (i = 0; i < MAX_N_PARAMS; i++) {
    NParamLabels[i]->setDisabled(true);
    NParamFields[i]->setDisabled(true);
  }

  NSendButton->setText(tr("Cancel"));
  NSendButton->setEnabled(true);
  ChangeImage();
  t->start(300);
}


//-------------------------------------------------------------
//
// TurnOff (private): called when the command has been sent,
//      or user has cancelled
//
//   *t: timer that was running animation
//
//-------------------------------------------------------------

void MainForm::TurnOff(QTimer *t) {
  int i;

  sending = false;

  NGroupsBox->setEnabled(true);
  NSettingsButton->setEnabled(true);
  NCurFileButton->setEnabled(true);
  NCurFile->setEnabled(true);
  NCommandList->setEnabled(true);
  NAboutLabel->setEnabled(true);
  for (i = 0; i < MAX_N_PARAMS; i++) {
    NParamLabels[i]->setEnabled(true);
    NParamFields[i]->setEnabled(true);
  }

  if (NCommandList->currentItem() == -1 || !NCommandList->hasFocus())
    NSendButton->setEnabled(true);
  NSendButton->setText(tr("Send"));
  ChangeImage();
  t->stop();
}


//-------------------------------------------------------------
//
// SendCommand (slot): triggered when user presses send button.
//    Forks off a process which runs blastcmd.
//
//-------------------------------------------------------------

void MainForm::SendCommand() {
  char tmp[12];
  int index, i, j;
  char args[MAX_N_PARAMS + 7][SIZE_NAME];
  char *params[MAX_N_PARAMS + 7];
  char buffer[1024];

  if (!sending) {
    i = 0;

    // Select appropiate flags
    strcpy(args[i++], BLASTCMDFILE);
    if (NVerbose->isChecked())
      strcpy(args[i++], "-v");
    strcpy(args[i++], "-f");
    strcpy(args[i++], "-s");
    switch (NSendMethod->currentItem()) {
      case 0:
        strcpy(args[i++], "-los");
        break;
      case 1:
        strcpy(args[i++], "-tdrss");
        break;
      case 2:
        strcpy(args[i++], "-hf_ptt");
        break;
    }
    switch (NSendRoute->currentItem()) {
      case 0:
        strcpy(args[i++], "-com1");
        break;
      case 1:
        strcpy(args[i++], "-com2");
        break;
    }

    // command name
    strcpy(args[i++], NCommandList->text(NCommandList->currentItem()));

    // Parameters
    if ((index = MIndex(NCommandList->text(NCommandList->currentItem())))
        != -1) {

      // Check to see if this command requires a confirm
      if (mcommands[index].group & CONFIRM) {
        sprintf(buffer, "The command %s requires confirmation.\n"
            "Are you sure you want to send this command?",
            NCommandList->text(NCommandList->currentItem()).ascii());

        if ( QMessageBox::warning(this, "Confirm Command", tr(buffer),
              QMessageBox::Yes, QMessageBox::Escape | QMessageBox::No |
              QMessageBox::Default) == QMessageBox::No ) {
          strcpy(buffer, "Command not confirmed.");
          params[1] = buffer;
          params[2] = 0;
          WriteCmd(NLog, params);
          WriteErr(NLog, 11);
          return;
        }
      }

      for (j = 0; j < mcommands[index].numparams; j++) {
        NParamFields[j]->RecordDefaults();
        if (defaults->asDouble(index, j) < mcommands[index].params[j].min) {
          sprintf(buffer, "Error: Parameter \"%s\" out of range: %f < %f",
              mcommands[index].params[j].name, defaults->asDouble(index, j),
              mcommands[index].params[j].min);
          params[1] = buffer;
          params[2] = 0;
          WriteCmd(NLog, params);
          WriteErr(NLog, 9);
          return;
        } else if (defaults->asDouble(index, j)
            > mcommands[index].params[j].max) {
          sprintf(buffer, "Error: Parameter \"%s\" out of range: %f > %f",
              mcommands[index].params[j].name, defaults->asDouble(index, j),
              mcommands[index].params[j].max);
          params[1] = buffer;
          params[2] = 0;
          WriteCmd(NLog, params);
          WriteErr(NLog, 9);
          return;
        }
        if (mcommands[index].params[j].type == 'i')
          sprintf(args[i++], "%i ", defaults->asInt(index, j));
        else
          sprintf(args[i++], "%f ", defaults->asDouble(index, j));
      }
    } else {
      index = SIndex(NCommandList->text(NCommandList->currentItem()));

      // Check to see if this command requires a confirm
      if (scommands[index].group & CONFIRM) {
        sprintf(buffer, "The command %s requires confirmation.\n"
            "Are you sure you want to send this command?",
            NCommandList->text(NCommandList->currentItem()).ascii());

        if ( QMessageBox::warning(this, "Confirm Command", tr(buffer),
              QMessageBox::Yes, QMessageBox::Escape | QMessageBox::No |
              QMessageBox::Default) == QMessageBox::No ) {
          strcpy(buffer, "Command not confirmed.");
          params[1] = buffer;
          params[2] = 0;
          WriteCmd(NLog, params);
          WriteErr(NLog, 11);
          return;
        }
      }
    }



    params[i] = NULL;
    for (j = 0; j < i; j++)
      params[j] = args[j];

    // Update log file on disk
    WriteCmd(NLog, params);
    WriteLog(params);

    sendingpid = fork();
    if (!sendingpid)
      execvp(BLASTCMDFILE, params);
    else
      TurnOn(timer);
  } else
    kill(sendingpid, SIGKILL);
}


//-------------------------------------------------------------
//
// ChangeCurFile (slot): triggered when the user enters a new
//      .cur file in the settings dialog window
//
//-------------------------------------------------------------

void MainForm::ChangeCurFile() {
  char txt[50], info[255];

  strcpy(txt, NCurFile->text());
  DataSource->~KstFile();
  DataSource = new KstFile(txt, UNKNOWN);
  sprintf(info, "Narsil will now read from %s.", txt);
  QMessageBox::information(this, "Acknowledgement", info,
      QMessageBox::Ok | QMessageBox::Default);
}


//-------------------------------------------------------------
//
// ShowSettings (slot): pops up the settings dialog window
//
//-------------------------------------------------------------

void MainForm::ShowSettings()
{
  NSettingsWindow->show();
}


//-------------------------------------------------------------
//
// WriteCmd (private): keeps a record of the commands sent
//      with Narsil; also records it in a textbox visible to
//      the user
//
//   *dest: the textbox
//   *args: the arguments sent to blastcmd
//
//-------------------------------------------------------------

void MainForm::WriteCmd(QMultiLineEdit *dest, char *args[]) {
  FILE *f;
  time_t t;
  char txt[2048];
  int i;

  t = time(NULL);
  f = fopen(LOGFILE, "a");

  if (f == NULL) {
    fprintf(stderr, "Narsil: could not write log file %s.\n", LOGFILE);
    return;
  }

  sprintf(txt, "%s", ctime(&t));
  fprintf(f, txt);
  txt[strlen(txt) - 1] = '\0';        // Get rid of the \n character
  dest->insertLine(txt);
  dest->setCursorPosition(dest->numLines() - 1, 0);

  i = 1;
  strcpy(txt, "  ");
  while (args[i] != '\0') {
    strcat(txt, args[i++]);
    strcat(txt, " ");
  }
  strcat(txt, "\n");

  fprintf(f, txt);
  txt[strlen(txt) - 1] = '\0';        // Get rid of the \n character
  dest->insertLine(txt);
  dest->setCursorPosition(dest->numLines() - 1, 0);
  fclose(f);
}

void MainForm::WriteLog(char *args[]) {
  QDateTime qdt;
  QString LogEntry, Group;
  int i, j;

  Group = GroupNames[GetGroup()];

  qdt = QDateTime::currentDateTime();

  LogEntry = NAboutLabel->text();
  LogEntry += "\n";

  if ((j = MIndex(NCommandList->text(NCommandList->currentItem())))
      != -1) {
    for (i=0; i < mcommands[j].numparams; i++) {
      LogEntry += QString("%1: %2").
                  arg(mcommands[j].params[i].name, 30).
                  arg(NParamFields[i]->text(), -8);
      if (i%2==1) LogEntry+= "\n";
    }
      LogEntry += "\n";
  }

  LogEntry+="\n";

  for (i=0; args[i] != '\0'; i++) {
    LogEntry += args[i];
    LogEntry += " ";
  }

  LogEntry+="\nTransmit via ";
  switch (NSendMethod->currentItem()) {
      case 0:
        LogEntry+="Line of Sight";
        break;
      case 1:
        LogEntry+="TDRSS";
        break;
      case 2:
        LogEntry+="HF radio";
        break;
  }
  LogEntry+= " through SIP ";
  switch (NSendRoute->currentItem()) {
      case 0:
        LogEntry+="COM 1";
        break;
      case 1:
        LogEntry+="COM 2";
        break;
  }

  LogEntry += "\n";
  LogEntry += QString(
    "-- Time  : %1\n"
    "-- Source: %2  Type: %3  Entry By: %4\n"
    "-- Frame : %5  File: %6  \n")
             .arg(qdt.toString())
             .arg("narsil",-10)
             .arg(Group, -10)
             .arg((getpwuid(getuid()))->pw_name, -10)
             .arg(DataSource->numFrames(),-10)
             .arg(DataSource->fileName());

  LogEntry+="--------------------------------------------------\n\n";

  QString logfilename = LOGFILEDIR +
                        qdt.toString("yyyy-MM-dd.hh:mm:ss") + "." +
                        "narsil." +
                        Group.replace(" ", "_")+ "." +
                        QString((getpwuid(getuid()))->pw_name);

  QFile logfile(logfilename);

  if (logfile.open(IO_WriteOnly)) {
    QTextStream stream(&logfile);
    stream << LogEntry;
    logfile.close();
  }

#ifdef USE_ELOG
  QString elog_command = QString(
    "elog -h " ELOG_HOST " -p 8080 -l blast-narsil "
    "-u narsil submmblast "
    "-a User=%1 -a Source=narsil -a Category=%2 -m %3 > /dev/null")
                         .arg(QString((getpwuid(getuid()))->pw_name))
                         .arg(Group)
                         .arg(logfilename);

  system(elog_command.latin1());
#endif
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

void MainForm::WriteErr(QMultiLineEdit *dest, int retstatus) {
  time_t t;
  QString txt;

  switch (retstatus) {
    case -1:
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
      txt = "  COMMAND POSSIBLY NOT SENT:  Received a garbage "
            "acknowledgement packet.\n";
      break;
    case 9:
      txt = "  COMMAND NOT SENT: Narsil error: Parameter out of range.\n";
      break;
    case 11:
      txt = "  COMMAND NOT SENT: Command not confirmed by user.\n";
      break;
    case 13:
      txt = "  COMMAND POSSIBLY NOT SENT: Timeout waiting for ack from GSE.\n";
      break;
  }

  QFile f(LOGFILE);
  if (f.open(IO_Append | IO_WriteOnly)) {
    QTextStream stream( &f );
    stream << txt;
    f.close();
  } else {
    fprintf(stderr, "Narsil: could not write log file %s.\n", LOGFILE);
    return;
  }

  dest->insertLine(txt);
  dest->setCursorPosition(dest->numLines() - 1, 0);
}


//-------------------------------------------------------------
//
// ReadLog (private): read in the log when Narsil starts
//
//   *dest: the textbox to write to
//
//-------------------------------------------------------------

void MainForm::ReadLog(QMultiLineEdit *dest) {
  FILE *f;
  char txt[255];

  f = fopen(LOGFILE, "r");
  dest->setText(" ");

  if (f == NULL) {
    printf("Narsil:  could not read log file %s.\n", LOGFILE);
    return;
  }

  while(fgets(txt, 255, f) != NULL) {
    txt[strlen(txt) - 1] = '\0'; // Get rid of the \n character from each line
    dest->insertLine(txt, dest->numLines());
  }
  dest->insertLine("", dest->numLines());
  dest->setCursorPosition(dest->numLines() - 1, 0);
}


//-------------------------------------------------------------
//
// MainForm: constructor
//
//-------------------------------------------------------------

MainForm::MainForm(char *cf, QWidget* parent,  const char* name, bool modal,
    WFlags fl)
: QDialog( parent, name, modal, fl ) {

  QFont tfont;
  int i;
  int indexes[25];
  char tmp[SIZE_NAME + SIZE_ABOUT + SIZE_PARNAME];
  QSize tempsize;
  QRect rect;
  QPoint point;
  int w1, w2, w3, h1, h2, h3;
  QString default_family = tfont.family();


  curfile = cf;
  lastmcmd = -1;
  sending = 0;
  sendingpid = -1;

  Images[0] = new QPixmap(DATA_DIR "/sword0.jpg");
  Images[1] = new QPixmap(DATA_DIR "/sword1.jpg");
  Images[2] = new QPixmap(DATA_DIR "/sword2.jpg");
  Images[3] = new QPixmap(DATA_DIR "/sword3.jpg");

  framenum = 0;
  numframes = 4;
  dir = 1;

  if ( !name )
    setName("Narsil");
  setCaption(tr("Narsil " VERSION));
  //  setIcon(*Icon);


  // Lay everything out.  Everything is very carefully positioned -- there are
  // no automatic spacers because with the dynamic properties of the program
  // (things popping in and out of existence as different commands are
  // chosen) things would mess up.  So the code ain't too pretty . . .
  NColorGroup = new QColorGroup(QColor("black"), QColor("lightGray"),
      QColor("white"), QColor(0x3f, 0x3f, 0x3f),
      QColor("darkGray"), QColor("black"),
      QColor("red"), QColor("white"), QColor("lightGray"));
  NColorGroup->setColor(QColorGroup::Highlight, "blue");
  NColorGroup->setColor(QColorGroup::HighlightedText, "white");
  NColorGroup2 = new QColorGroup(*NColorGroup);
  NColorGroup2->setColor(QColorGroup::ButtonText, "darkGray");

  NGroupsBox = new QButtonGroup(this, "NGroupsBox");
  NGroupsBox->setColumnLayout(0, Qt::Vertical);
  NGroupsBox->layout()->setSpacing(0);
  NGroupsBox->layout()->setMargin(0);

  NGroupsLayout = new QGridLayout(NGroupsBox->layout());
  NGroupsLayout->setAlignment(Qt::AlignTop);
  NGroupsLayout->setSpacing(1);
  NGroupsLayout->setMargin(5);

  for (i = 0; i < N_GROUPS; i++) {
    NGroups[i] = new QRadioButton(NGroupsBox, "QGroup");
    NGroups[i]->setText(tr(GroupNames[i]));
    tempsize = NGroups[i]->sizeHint();
    NGroupsLayout->addWidget(NGroups[i], int(i/3), (i%3));
  }

  NGroups[0]->setChecked(true);
  connect(NGroupsBox, SIGNAL(clicked(int)), this, SLOT(ChangeCommandList()));

  NTopFrame = new QFrame(this, "NTopFrame");
  NTopFrame->setFrameShape(QFrame::Box);
  NTopFrame->setFrameShadow(QFrame::Sunken);

  NCommandList = new QListBox(this, "NCommandList");
  NCommandList->adjustSize();
  NCommandList->setGeometry(PADDING, PADDING, NCommandList->width() + 80, 0);
  connect(NCommandList, SIGNAL(highlighted(int)), this, SLOT(ChooseCommand()));


  w1 = 0;

  strcpy(tmp, LongestParam());
  for (i = 0; i < MAX_N_PARAMS; i++) {
    NParamLabels[i] = new QLabel(NTopFrame, "NParamLabel");
    NParamLabels[i]->setText(tr(tmp));
    NParamLabels[i]->setAlignment(Qt::AlignHCenter);
    NParamLabels[i]->adjustSize();
    w2 = NParamLabels[i]->width();

    NParamFields[i] = new DoubleEntry(NTopFrame, "NParamLabels");
    NParamFields[i]->setFixedWidth(w2/2);
    NParamFields[i]->adjustSize();

    w3 = NParamFields[i]->width();
    h2 = NParamLabels[i]->height();
    h3 = NParamFields[i]->height();

    point.setX(w1 + 2 * PADDING + (i%2) * (w2 + w3 + PADDING));
    NParamLabels[i]->setGeometry(point.x(), 0, w2, h2);

    point.setX(w1 + PADDING +
        (i%2) * (w2 + w3 + PADDING) + w2);
    NParamFields[i]->setGeometry(point.x(), 0, w3, h3);
  }


  memset(tmp, 'B', SIZE_ABOUT);
  tmp[SIZE_ABOUT] = '\0';

  NAboutLabel = new QLabel(NTopFrame, "NAboutLabel");
  NAboutLabel->setFrameShape(QFrame::Box);
  NAboutLabel->setFrameShadow(QFrame::Plain);
  NAboutLabel->setText(tr(tmp));
  NAboutLabel->setAlignment(int(QLabel::WordBreak | QLabel::AlignLeft));
  NAboutLabel->setGeometry(0, 0, 2 * (w2 + w3 + PADDING) + SPACING * 4, 0);
  tempsize = NAboutLabel->sizeHint();
  NAboutLabel->setGeometry(PADDING , PADDING,
      2 * (w2 + w3 + PADDING) + SPACING * 4,
      tempsize.height());

  h1 = NAboutLabel->height();
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

  NSendButton = new QPushButton(NTopFrame, "NSendButton");
  NSendButton->setText(tr("Cancel"));
  NSendButton->adjustSize();
  NSendButton->setDisabled(true);
  NSettingsButton = new QPushButton(NTopFrame, "NSettingsButton");
  NSettingsButton->setText(tr("Settings . . ."));
  NSettingsButton->adjustSize();

  NSettingsButton->setGeometry(PADDING, PADDING +
      2 * PADDING + h1 +
      (int((2 + MAX_N_PARAMS)/2)) *
      (h3 + SPACING) -
      NSettingsButton->height()
      , NSettingsButton->width(),
      NSettingsButton->height());
  NSendButton->setGeometry(2*PADDING + NAboutLabel->width() -
      NSendButton->width(),
      PADDING + 2 * PADDING + h1 +
      (int((2 + MAX_N_PARAMS)/2)) *
      (h3 + SPACING) -
      NSendButton->height(),
      NSendButton->width(),
      NSendButton->height());

  NTopFrame->adjustSize();

  NGroupsBox->adjustSize();
  NGroupsBox->setGeometry(2*PADDING+NCommandList->width(), PADDING,
      NTopFrame->width(),
      NGroupsBox->height());

  NTopFrame->setGeometry(2*PADDING+NCommandList->width(),
      PADDING * 2 + NGroupsBox->height(),
      NTopFrame->width(), NTopFrame->height());


  NBotFrame = new QFrame(this, "NBotFrame");
  NBotFrame->setFrameShape(QFrame::Box);
  NBotFrame->setFrameShadow(QFrame::Sunken);

  NWaitImage = new QLabel(NBotFrame, "NWaitImage");
  NWaitImage->setScaledContents(false);
  ChangeImage();
  NWaitImage->adjustSize();
  NWaitImage->setGeometry(NTopFrame->width() - NWaitImage->width() - PADDING,
      PADDING, NWaitImage->width(), NWaitImage->height());


  NLog = new QMultiLineEdit(NBotFrame, "NLog");
  NLog->setReadOnly(true);
  NLog->setGeometry(PADDING, PADDING, NTopFrame->width() -
      NWaitImage->width() -
      3 * PADDING, PADDING + NWaitImage->height());
  ReadLog(NLog);

  NBotFrame->adjustSize();
  NBotFrame->setGeometry(2*PADDING+NCommandList->width(),
      PADDING * 3 + NGroupsBox->height() +
      NTopFrame->height(), NTopFrame->width(),
      NBotFrame->height());

  NCommandList->setGeometry(PADDING, PADDING, NCommandList->width(),
      PADDING * 2 + NGroupsBox->height() +
      NTopFrame->height() + NBotFrame->height());

  // Settings window
  NSettingsWindow = new QDialog(this, "NSettingsWindow", true, 0);
  NSettingsWindow->setName("SettingsWindow");
  NSettingsWindow->setCaption(tr("Narsil Settings"));

  NCurFileCaption = new QLabel(NSettingsWindow, "NCurFileCaption");
  NCurFileCaption->setText(tr("Cur File: "));
  NCurFileCaption->adjustSize();

  NCurFile = new QLineEdit(NSettingsWindow, "NCurFile");
  NCurFile->setText(tr(curfile));
  NCurFile->adjustSize();

  NCurFileButton = new QPushButton(NSettingsWindow, "QCurCaption");
  NCurFileButton->setText(tr("Change"));
  NCurFileButton->adjustSize();

  NVerbose = new QCheckBox(NSettingsWindow, "NVerbose");
  NVerbose->setText(tr("-v (verbose)"));
  NVerbose->setChecked(false);
  NVerbose->adjustSize();
  NVerbose->setGeometry(PADDING, 2 * PADDING + NCurFileButton->height(),
      NVerbose->width(), NVerbose->height());

  NSendMethod = new QComboBox(NSettingsWindow, "NSendMethod");
  NSendMethod->insertItem(tr("LOS"));
  NSendMethod->insertItem(tr("TDRSS"));
  NSendMethod->insertItem(tr("HF PTT"));
  NSendMethod->adjustSize();
  NSendMethod->setGeometry(2 * PADDING + NVerbose->width(),
      2 * PADDING + NCurFileButton->height(),
      NSendMethod->width(), NSendMethod->height());

  NSendRoute = new QComboBox(NSettingsWindow, "NSendRoute");
  NSendRoute->insertItem(tr("COMM 1"));
  NSendRoute->insertItem(tr("COMM 2"));
  NSendRoute->adjustSize();
  NSendRoute->setGeometry(3 * PADDING + NSendMethod->width() +
      NVerbose->width(),
      2 * PADDING + NCurFileButton->height(),
      NSendRoute->width(), NSendRoute->height());

  NCloseSettingsWindow = new QPushButton(NSettingsWindow,
      "NCloseSettingsWindow");
  NCloseSettingsWindow->setText(tr("Close"));
  NCloseSettingsWindow->adjustSize();
  NCloseSettingsWindow->setGeometry(0, 4 * PADDING + NCurFileButton->height()
      + NSendRoute->height(),
      NCloseSettingsWindow->width(),
      NCloseSettingsWindow->height());

  NTSettingsLayout = new QHBoxLayout();
  NTSettingsLayout->setSpacing(3);
  NTSettingsLayout->setMargin(11);
  NTSettingsLayout->addWidget(NCurFileCaption);
  NTSettingsLayout->addWidget(NCurFile);
  NTSettingsLayout->addWidget(NCurFileButton);

  NMSettingsLayout = new QHBoxLayout();
  NMSettingsLayout->setSpacing(3);
  NMSettingsLayout->setMargin(11);
  NMSettingsLayout->addWidget(NVerbose);
  NMSettingsLayout->addWidget(NSendMethod);
  NMSettingsLayout->addWidget(NSendRoute);

  NBSettingsLayout = new QHBoxLayout();
  NBSettingsLayout->setSpacing(3);
  NBSettingsLayout->setMargin(11);
  NBSettingsLayout->addWidget(NCloseSettingsWindow);

  NSettingsLayout = new QVBoxLayout(NSettingsWindow);
  NSettingsLayout->setSpacing(0);
  NSettingsLayout->setMargin(0);
  NSettingsLayout->addItem(NTSettingsLayout);
  NSettingsLayout->addItem(NMSettingsLayout);
  NSettingsLayout->addItem(NBSettingsLayout);

  NSettingsWindow->adjustSize();
  NSettingsWindow->setMinimumSize(QSize(NSettingsWindow->width(),
        NSettingsWindow->height()));
  NSettingsWindow->setMaximumSize(QSize(NSettingsWindow->width(),
        NSettingsWindow->height()));

  adjustSize();
  setMinimumSize(QSize(width(), height()));
  setMaximumSize(QSize(width(), height()));

  if (!curfile.isNull())
    strcpy(tmp, curfile);
  else
    strcpy(tmp, '\0');

  DataSource = new KstFile(tmp, UNKNOWN);
  DataSource->update();

  timer = new QTimer(this, "timer");
  TurnOff(timer);

  ChangeCommandList();
  ChooseCommand();

  connect(NSendButton, SIGNAL(clicked()), this, SLOT(SendCommand()));
  connect(NCurFileButton, SIGNAL(clicked()), this, SLOT(ChangeCurFile()));
  connect(NSettingsButton, SIGNAL(clicked()), this, SLOT(ShowSettings()));
  connect(NCloseSettingsWindow, SIGNAL(clicked()), NSettingsWindow,
      SLOT(accept()));
  connect(timer, SIGNAL(timeout()), this, SLOT(ChangeImage()));
}

MainForm::~MainForm()
{
  // no need to delete child widgets, Qt does it all for us
}




//|||****_______________________________________________________________________
//|||***************************************************************************
//|||****
//|||****
//|||****     Main()
//|||****
//|||****
//|||****_______________________________________________________________________
//|||***************************************************************************

Defaults *defaults;
int main(int argc, char* argv[]) {
  int pipefd[2];
  int i, j;
  char buffer[80]; 
  char* ptr;

  if (argc > 1) {
    printf(
        "Narsil " VERSION " doesn't take arguments.  It was compiled at "
        __DATE__ "\nand is copyright (C) 2002-2004 University of Toronto.\n\n"
        "This program comes with NO WARRANTY, not even for MERCHANTABILITY or "
        "FITNESS\n"
        "FOR A PARTICULAR PURPOSE. You may redistribute it under the terms of "
        "the GNU\n"
        "General Public License; see the file named COPYING for details.\n\n"
        "Narsil was written by Adam Hincks. It was later poked at a bit by "
        "D.V. Wiebe\n"
        "and then further desecrated by cbn.\n"
        );
    exit(1);
  }

  /* begin blastcmd command list revision check */ 
  if (pipe(pipefd)) {
    perror("pipe");
    exit(1);
  }

  if ((i = fork()) == -1) {
    perror("fork");
    exit(1);
  } else if (i == 0) {
    close(1);
    dup2(pipefd[1], 1);
    close(pipefd[0]);
    freopen("/dev/null", "w", stderr);
    if (execlp(BLASTCMDFILE, BLASTCMDFILE, "-c", NULL)) {
      perror(BLASTCMDFILE);
      exit(1);
    }
  }
  close(pipefd[1]);
  j = read(pipefd[0], buffer, 80);
  if (j == -1) {
    perror("pipe read");
    j = 0;
  }

  buffer[j] = '\n';
  *strchr(buffer, '\n') = '\0';
  
  if ((ptr = strchr(buffer, '$')) == NULL)
    ptr = buffer;

  wait(&i);
  if (!WIFEXITED(i)) {
    printf("`%s -c' terminated abnormally; cannot continue.\n", BLASTCMDFILE);
    exit(1);
  }

  close(pipefd[0]);
  if (WEXITSTATUS(i) == 12 || !strncmp(ptr, "$Revision: ", 11)) {
    if (strncmp(command_list_serial, ptr, strlen(command_list_serial))) {
      printf("\n                           **** ERROR ****\n\n"
          "Cowardly refusing to use a blastcmd with a different command list.\n"
          "command list revision for %s:\n  %s\n"
          "command list revision for narsil:\n  %s\n", BLASTCMDFILE, ptr,
          command_list_serial);
      exit(1);
    }
  } else {
    printf("\n                          **** WARNING ****\n\n"
        "The blastcmd program I'm attempting to use (%s)\n"
        "doesn't support command list revsion reporting.  Be certain that the\n"
        "command list used by narsil is identical to the one used by blastcmd\n"
        "before using this program.\n", BLASTCMDFILE);
  }
  /* end blastcmd command list revision check */ 

  defaults = new Defaults();

  QApplication app(argc, argv);
  MainForm narsil(DEF_CURFILE, 0, "narsil", true, 0);

  app.setMainWidget(&narsil);

  int ret = narsil.exec();
  return ret;
}
