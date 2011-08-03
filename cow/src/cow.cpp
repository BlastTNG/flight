/* cow (previously known as narsil): GUI commanding front-end
 *
 * This software is copyright (C) 2002-2006 University of Toronto
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
// *  "Ported" to qt4 by drmrshdw                    *
// ***************************************************

#define _XOPEN_SOURCE 501

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

#define VERSION "4.0-testing"   // don't remove "testing" until you're sure it works reliably...

#ifndef BLASTCMD_HOST
#error Run ./configure
#endif

#ifndef CUR_DIR
#error Run ./configure
#endif

#ifndef DATA_ETC_NARSIL_DIR
#error Run ./configure
#endif

#define PADDING 3
#define SPACING 3

#define DEF_CURFILE CUR_DIR "/defile.lnk"
#define LOGFILE DATA_ETC_NARSIL_DIR "/log.txt"
#define LOGFILEDIR DATA_ETC_NARSIL_DIR "/log/"

#define DATA_DIR DATA_ETC_NARSIL_DIR

#ifndef ELOG_HOST
#define ELOG_HOST "elog.blast"
#endif

#define PING_TIME 59000
#define WAIT_TIME 40

const char* blastcmd_host;

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
// Defaults::Save: Narsil remembers any values entered for the
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
    strncpy(sdefaults[j][i], text.toAscii(), CMD_STRING_LEN - 1);
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

    for (i = 0; i < client_n_groups; i++)
        if (NGroups[i]->isChecked())
            return i;

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

void MainForm::OmniParse(QString x)
{
    x=(x=="__AUTODETECT__")?NOmniBox->text():x;

    if(NOmniBox->cursorPosition()&&x[NOmniBox->cursorPosition()-1]==' ') {
        int nxt=NOmniBox->text().indexOf(' ',NOmniBox->cursorPosition()+1);
        NOmniBox->setSelection(NOmniBox->cursorPosition(),(nxt==-1)?999999:nxt-NOmniBox->cursorPosition());
    }

    if(x.contains(' ')) {
        QStringList words=x.split(" ");
        for(int i=1;i<words.size();i++) {
            if(NParamFields[i-1]->isVisible()&&!words[i].isEmpty()) {
                words[i].replace("..",".");
                bool ok=dynamic_cast<NarsilStringEntry*>(NParamFields[i-1]);
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
                    dynamic_cast<AbstractNarsilEntry*>(NParamFields[i-1])->SetStringValue(words[i]);
                }
            }
        }
        x.truncate(x.indexOf(' ')+1);
        for(int i=0;i<MAX_N_PARAMS;i++) {
            if(NParamFields[i]->isVisible()) {
                x.append(i?" ":"");
                x.append(dynamic_cast<AbstractNarsilEntry*>(NParamFields[i])->Text());
            }
        }
        int cp=NOmniBox->cursorPosition();
        int sbeg=NOmniBox->selectionStart();
        int slen=NOmniBox->selectedText().size();
        if(x!=NOmniBox->text()) {
            NOmniBox->setText(x);
        }
        NOmniBox->setCursorPosition(cp);
        if(sbeg!=-1) {
            NOmniBox->setSelection(sbeg,slen);
        }
        return;
    }

    if(x.contains(' ')&&NOmniBox->cursorPosition()<x.indexOf(' ')) {
        x.truncate(x.indexOf(' ')-1);
        NOmniBox->setText(x);
        NOmniBox->completer()->setCompletionPrefix(x);
        NOmniBox->completer()->complete();
        return;
    }
    if(sender()!=this&&!x.contains(' ')&&!x.isEmpty()) {
        bool changed=0;
        for(int i=0;i<OmniList.size();i++) {
            if(x==OmniList[i].name) {
                if(!NGroups[OmniList[i].group]->isChecked()) {
                    NGroups[OmniList[i].group]->toggle();
                }
                changed=1;
                break;
            }
        }
    }

    if(!x.contains(' ')) {
        for(int i=0;i<NCommandList->count();i++) {
            //NCommandList->item(i)->setHidden( !NCommandList->item(i)->text().contains(x) );
            if(NCommandList->item(i)->text()==x) {
                NCommandList->setCurrentRow(i);
            }
        }
    }
}

void MainForm::OmniSync()
{
    QString x=NOmniBox->text();
    QStringList words=x.split(" ");
    for(int i=1;i<words.size();i++) {
        if(NParamFields[i-1]->isVisible()&&!words[i].isEmpty()) {
            words[i].replace("..",".");
            words[i]=dynamic_cast<AbstractNarsilEntry*>(NParamFields[i-1])->Text();\
        }
    }
    int cp=NOmniBox->cursorPosition();
    int sbeg=NOmniBox->selectionStart();
    int slen=NOmniBox->selectedText().size();
    NOmniBox->setText(words.join(" "));
    NOmniBox->setCursorPosition(cp);
    if(sbeg!=-1) {
        NOmniBox->setSelection(sbeg,slen);
    }
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

    max = GroupSIndexes(GetGroup(), indexes);
    max += GroupMIndexes(GetGroup(), &indexes[max]);
    qsort(indexes, max, sizeof(int), &TheSort);

    for (i = 0; i < max; i++)
        if (indexes[i] >= client_n_scommands)
            NCommandList->addItem(client_mcommands[indexes[i]
                                  - client_n_scommands].name);
        else
            NCommandList->addItem(client_scommands[indexes[i]].name);

    ChooseCommand();
    OmniParse();
}

//-------------------------------------------------------------
// ChooseCommand (slot): triggered when a command is selected
//      from the list. Changes labels and shows spin-boxes
//      as appropiate for the command.
//-------------------------------------------------------------

void MainForm::ChooseCommand() {
    int i, index;
    double indata;

    delete _dirfile;
    _dirfile = new Dirfile(_curFileName, GD_RDONLY);

    // Remember parameter values
    if (lastmcmd != -1) {
        for (i = 0; i < client_mcommands[lastmcmd].numparams; i++)
            dynamic_cast<AbstractNarsilEntry*>(NParamFields[i])->RecordDefaults();

        defaults->Save();
    }

    // It can happen that this function be called with nothing selected
    if (!NCommandList->currentIndex().isValid() || (!NCommandList->hasFocus()&&!NOmniBox->hasFocus())) {
        NSendButton->setDisabled(true);
        lastmcmd = -1;
        NAboutLabel->setText(tr("No command selected."));
        for (i = 0; i < MAX_N_PARAMS; i++) {
            NParamLabels[i]->hide();
            NParamFields[i]->hide();
        }
    } else {
        NSendButton->setEnabled(true);
        NOmniBox->setText(NCommandList->currentItem()->text()+" ");
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

            for (i = 0; i < MAX_N_PARAMS; i++) {
                if (i < client_mcommands[index].numparams) {
                    NParamLabels[i]->setText(client_mcommands[index].params[i].name);
                    NParamLabels[i]->show();

                    bool typeChanged=0;
                    QRect geometry=NParamFields[i]->geometry();
                    if(dynamic_cast<NarsilDoubleEntry*>(NParamFields[i])&&client_mcommands[index].params[i].type=='s')
                    {
                        typeChanged=1;
                        delete NParamFields[i];
                        connect(NParamFields[i]=new NarsilStringEntry(NTopFrame,"NParamLabels"),
                                SIGNAL(textEdited(QString)),this,SLOT(OmniSync()));
                    }
                    else if(dynamic_cast<NarsilStringEntry*>(NParamFields[i])&&client_mcommands[index].params[i].type!='s')
                    {
                        typeChanged=1;
                        delete NParamFields[i];
                        NParamFields[i]=new NarsilDoubleEntry(NTopFrame,"NParamLabels");
                        connect(dynamic_cast<NarsilDoubleEntry*>(NParamFields[i]),
                                SIGNAL(valueChanged(double)),this,SLOT(OmniSync()));
                    }
                    if(typeChanged)
                    {
                        int w2 = NParamLabels[i]->width();

                        NParamFields[i]->setFixedWidth(w2/2);
                        NParamFields[i]->adjustSize();

                        NParamFields[i]->setGeometry(geometry);
                    }
                    NParamFields[i]->show();
                    dynamic_cast<AbstractNarsilEntry*>(NParamFields[i])->SetParentField(index, i);
                    dynamic_cast<AbstractNarsilEntry*>(NParamFields[i])->SetType(client_mcommands[index].params[i].type);
                    if (client_mcommands[index].params[i].type == 's') {
                        dynamic_cast<AbstractNarsilEntry*>(NParamFields[i])->SetStringValue(
                                    client_mcommands[index].params[i].field);
                    } else {
                        int nf;
                        if ((nf = _dirfile->NFrames())>0) {
                            if (_dirfile->GetData( client_mcommands[index].params[i].field,
                                                  nf-1, 0, 0, 1, // 1 sample from frame nf-1
                                                  Float64, (void*)(&indata))==0) {
                                dynamic_cast<AbstractNarsilEntry*>(NParamFields[i])->SetDefaultValue(index, i);
                            } else {
                                dynamic_cast<AbstractNarsilEntry*>(NParamFields[i])->SetValue(indata);
                            }
                        } else {
                            dynamic_cast<AbstractNarsilEntry*>(NParamFields[i])->SetDefaultValue(index, i);
                        }
                    }
                } else {
                    NParamLabels[i]->hide();
                    NParamFields[i]->hide();
                }
            }
        }
        OmniParse();
        NOmniBox->setFocus();
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
        if (strcmp(client_scommands[i].name, cmd.toAscii()) == 0)
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
        if (strcmp(client_mcommands[i].name, cmd.toAscii()) == 0)
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
    int i;

    // Remember parameter values
    if (lastmcmd != -1) {
        for (i = 0; i < client_mcommands[lastmcmd].numparams; i++)
            dynamic_cast<AbstractNarsilEntry*>(NParamFields[i])->RecordDefaults();

        defaults->Save();
    }

    // Release control of program; give it back to Main()
    exit(0);
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

    if (sending) {
        if (framenum == 0 && dir == -1)
            dir = 1;
        framenum += dir;
        if (framenum == numframes - 1)
            dir = -1;
        if (framenum == 1)
            dir = 1;
        NWaitImage->setPixmap(*Images[framenum]);
    } else {
        if (framenum != 0) {
            framenum+=dir;
            if(framenum==numframes-1) dir=-1;
            NWaitImage->setPixmap(*Images[framenum]);
        }
    }

    ReceivePackets(!verbose, CMD_NONE);

    ConnBanner->setText(NetCmdBanner());
    timer->start(WAIT_TIME);
}

int MainForm::ReceivePackets(int silent, int wait_for)
{
    Q_UNUSED(silent);
    int returnstatus;

    do {
        returnstatus = NetCmdReceive(!verbose);
        if (sending && ((returnstatus & 0xFF) == CMD_BCMD))
        {
            if ((returnstatus >> 8) == 12)
                returnstatus = (99 << 8);
            WriteErr(NLog, returnstatus >> 8);
            TurnOff();
        }
    } while (returnstatus != CMD_NONE && returnstatus != wait_for);
    return returnstatus;
}

void MainForm::Ping(void) {
    ping_timer->stop();
    if (!NetCmdPing())
    {
        printf("Error Sending Ping!\n");
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
    int i;

    sending = true;

    NGroupsBox->setDisabled(true);
    NCurFileButton->setDisabled(true);
    NCurFile->setDisabled(true);
    NCommandList->setDisabled(true);
    NAboutLabel->setDisabled(true);
    for (i = 0; i < MAX_N_PARAMS; i++) {
        NParamLabels[i]->setDisabled(true);
        NParamFields[i]->setDisabled(true);
    }

    NSendButton->setText(tr("Cancel"));
    NSendButton->setEnabled(true);
}


//-------------------------------------------------------------
//
// TurnOff (private): called when the command has been sent,
//      or user has cancelled
//
//-------------------------------------------------------------

void MainForm::TurnOff(void) {
    int i;

    sending = false;

    NGroupsBox->setEnabled(true);
    NCurFileButton->setEnabled(true);
    NCurFile->setEnabled(true);
    NCommandList->setEnabled(true);
    NAboutLabel->setEnabled(true);
    for (i = 0; i < MAX_N_PARAMS; i++) {
        NParamLabels[i]->setEnabled(true);
        NParamFields[i]->setEnabled(true);
    }

    if (!NCommandList->currentIndex().isValid() || !NCommandList->hasFocus())
        NSendButton->setEnabled(true);
    NSendButton->setText(tr("Send (Shift+F12)"));
}


//-------------------------------------------------------------
//
// SendCommand (slot): triggered when user presses send button.
//
//-------------------------------------------------------------

void MainForm::SendCommand() {
    int index, i, j;
    char buffer[1024];
    char request[1024];
    const char link[] = "LTI";
    const char route[] = "12";

    if (!sending) {

        i = 0;

        // Select appropiate flags
        verbose = NVerbose->isChecked();

        request[0] = link[NSendMethod->currentIndex()];
        request[1] = route[NSendRoute->currentIndex()];
        request[2] = ' ';

        // command name
        strcpy(request + 3, NCommandList->currentItem()->text().toAscii());

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
                dynamic_cast<AbstractNarsilEntry*>(NParamFields[j])->RecordDefaults();
                if (defaults->asDouble(index, j)
                        < client_mcommands[index].params[j].min)
                {
                    sprintf(buffer, "Error: Parameter \"%s\" out of range: %f < %f",
                            client_mcommands[index].params[j].name, defaults->asDouble(index,
                                                                                       j), client_mcommands[index].params[j].min);
                    WriteCmd(NLog, buffer);
                    WriteErr(NLog, 10);
                    return;
                } else if (defaults->asDouble(index, j)
                           > client_mcommands[index].params[j].max) {
                    sprintf(buffer, "Error: Parameter \"%s\" out of range: %f > %f",
                            client_mcommands[index].params[j].name,
                            defaults->asDouble(index, j),
                            client_mcommands[index].params[j].max);
                    WriteCmd(NLog, buffer);
                    WriteErr(NLog, 10);
                    return;
                }
                if (client_mcommands[index].params[j].type == 'i'
                        || client_mcommands[index].params[j].type == 'l')
                    sprintf(buffer, " %i", defaults->asInt(index, j));
                else if (client_mcommands[index].params[j].type == 's')
                    sprintf(buffer, " %s", defaults->asString(index, j));
                else
                    sprintf(buffer, " %f", defaults->asDouble(index, j));
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

        /* Take the conn */
        /* note: loop takes place in TakeConn now. TODO implement limited attempts
    int times = 0;
    if (!NetCmdTakeConn()) {
      do {
        usleep(WAIT_TIME / 2 * 1000); //Half the normal wait time
        times++;
      } while (times < 25 && ReceivePackets(!verbose, CMD_CONN) != CMD_CONN);
      if (!NetCmdTakeConn()) { //Never got the conn.
        WriteCmd(NLog, request);
        WriteErr(NLog, 12);
        return;
      }
    }
    */
        if (!NetCmdTakeConn(verbose)) { //Never got the conn.
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
        NetCmdSend("::kill::\r\n");
    }
}


//-------------------------------------------------------------
//
// ChangeSettingsLabel (slot): triggered when user closes
//      settings dialog. Updates informative new label
//
//-------------------------------------------------------------

void MainForm::ChangeSettingsLabel() {
}


//-------------------------------------------------------------
//
// ChangeCurFile (slot): triggered when the user enters a new
//      .cur file in the settings dialog window
//
//-------------------------------------------------------------

void MainForm::ChangeCurFile() {
    QString msg;
    //as default path, use directory above current dirfile
    QDir dir(NCurFile->text());
    dir.cdUp();

    msg = NCurFile->text().toAscii();
    NCurFile->setText(QFileDialog::getExistingDirectory(this,"Select DirFile", dir.absolutePath(),0));
    if (!NCurFile->text().isEmpty()) {
        strcpy(_curFileName, NCurFile->text().toAscii());
        msg.sprintf("Narsil will now read from %s.", _curFileName);
        QMessageBox::information(this, "Acknowledgement", msg,
                                 QMessageBox::Ok | QMessageBox::Default);
    } else {
        NCurFile->setText(msg);
    }
}


//-------------------------------------------------------------
//
// ShowSettings (slot): pops up the settings dialog window
//
//-------------------------------------------------------------

void MainForm::ShowSettings()
{
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

void MainForm::WriteCmd(QTextEdit *dest, const char *request) {
    FILE *f;
    time_t t;
    char txt[2048];

    t = time(NULL);
    f = fopen(LOGFILE, "a");

    if (f == NULL) {
        fprintf(stderr, "Narsil: could not write log file %s.\n", LOGFILE);
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
                    arg(dynamic_cast<AbstractNarsilEntry*>(NParamFields[i])->Text(), -8);
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
    }
    LogEntry+= " through SIP ";
    switch (NSendRoute->currentIndex()) {
    case 0:
        LogEntry+="COMM 1";
        break;
    case 1:
        LogEntry+="COMM 2";
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
            .arg(_dirfile->NFrames(),-10)
            .arg(_dirfile->Name());

    LogEntry+="--------------------------------------------------\n\n";

    QString logfilename = LOGFILEDIR +
            qdt.toString("yyyy-MM-dd.hh:mm:ss") + "." +
            "narsil." +
            Group.replace(" ", "_")+ "." +
            QString((getpwuid(getuid()))->pw_name);

    QFile logfile(logfilename);

    if (logfile.open(QFile::WriteOnly)) {
        QTextStream stream(&logfile);
        stream << LogEntry;
        logfile.close();
    }

#ifdef USE_ELOG
    if (fork() == 0) {
        QString elog_command = QString(
                    ELOG " -h " ELOG_HOST " -p " ELOG_PORT " -l blast-narsil "
                    "-u narsil submmblast "
                    "-a User=%1 -a Source=narsil -a Category=%2 -m %3 > /dev/null 2>&1")
                .arg(QString((getpwuid(getuid()))->pw_name))
                .arg(Group)
                .arg(logfilename);

        if (system(elog_command))
            perror("Unable to exec " ELOG);
        exit(0);
    }
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

void MainForm::WriteErr(QTextEdit *dest, int retstatus) {
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
        txt = "  COMMAND NOT SENT: Narsil error: Parameter out of range.\n";
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
    }

    QFile f(LOGFILE);
    if (f.open(QFile::Append | QFile::WriteOnly)) {
        QTextStream stream( &f );
        stream << txt;
        f.close();
    } else {
        fprintf(stderr, "Narsil: could not write log file %s.\n", LOGFILE);
        return;
    }

    dest->insertPlainText("\n"+QString(txt));
    dest->moveCursor(QTextCursor::End);
}


//-------------------------------------------------------------
//
// ReadLog (private): read in the log when Narsil starts
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
        printf("Narsil:  could not read log file %s.\n", LOGFILE);
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
    if(ev->key()==Qt::Key_F12&&(ev->modifiers()&Qt::ShiftModifier)) {
        NSendButton->animateClick(100);
    } else if(dynamic_cast<AbstractNarsilEntry*>(focusWidget())) {
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
    NOmniBox->clear();
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
            if(!OmniList.contains(pair)) {
                OmniList.push_back(pair);
            }
        }
    }

    qSort(OmniList);

    QStringList sl;
    for(int i=0;i<OmniList.size();i++)
    {
        sl.push_back(OmniList[i].name);
    }

    if(NOmniBox->completer()) {
        delete NOmniBox->completer();
    }

    NOmniBox->setCompleter(new QCompleter(sl));
}

//-------------------------------------------------------------
//
// MainForm: constructor
//
//-------------------------------------------------------------

MainForm::MainForm(const char *cf, QWidget* parent,  const char* name,
                   Qt::WFlags fl) : QMainWindow( parent, fl )
{
    setObjectName(name?QString(name):"Narsil");
    int i;
    char tmp[SIZE_NAME + SIZE_ABOUT + SIZE_PARNAME];
    QSize tempsize;
    QPoint point;
    int w1, w2, w3, h1, h2, h3;

    _dirfile = NULL;

    centralWidget = new QWidget();
    theHLayout = new QHBoxLayout;
    theHLayout->addLayout(theVLayout = new QVBoxLayout());

    curfile = cf;
    lastmcmd = -1;
    sending = 0;

    Images[0] = new QPixmap(":/icons/cow0.png");
    Images[1] = new QPixmap(":/icons/cow1.png");
    Images[2] = new QPixmap(":/icons/cow2.png");
    Images[3] = new QPixmap(":/icons/cow3.png");
    Images[4] = new QPixmap(":/icons/cow4.png");
    Images[5] = new QPixmap(":/icons/cow5.png");

    framenum = 0;
    numframes = 4;
    dir = 1;

    strcpy(tmp, "Command Operations Window " VERSION " @");
    strcat(tmp, blastcmd_host);

    setWindowTitle(tmp);

    NOmniBox = new NarsilOmniBox(this->centralWidget);
    NOmniBox->setObjectName("NFilter");
    NOmniBox->adjustSize();
    //NOmniBox->setPlaceholderText("Awesome Bar");
    connect(NOmniBox,SIGNAL(textChanged(QString)),this,SLOT(OmniParse(QString)));
    connect(NOmniBox,SIGNAL(cursorPositionChanged(int,int)),this,SLOT(OmniParse()));
    theHLayout->addWidget(NOmniBox);

    // Lay everything out.  Everything is very carefully positioned -- there are
    // no automatic spacers because with the dynamic properties of the program
    // (things popping in and out of existence as different commands are
    // chosen) things would mess up.  So the code ain't too pretty . . .
    NGroupsBox = new QGroupBox(this->centralWidget);
    NGroupsBox->setObjectName("NGroupsBox");
    theVLayout->addWidget(NGroupsBox);

    NGroupsLayout = new QGridLayout(NGroupsBox);
    NGroupsLayout->setAlignment(Qt::AlignTop);
    NGroupsLayout->setSpacing(1);
    NGroupsLayout->setMargin(5);

    NGroups = new QRadioButton*[client_n_groups];
    for (i = 0; i < client_n_groups; i++) {
        NGroups[i] = new QRadioButton(NGroupsBox);
        connect(NGroups[i],SIGNAL(clicked()),NOmniBox,SLOT(clear()));
        connect(NGroups[i],SIGNAL(toggled(bool)),this,SLOT(ChangeCommandList(bool)));
        NGroups[i]->setObjectName("QGroup");
        NGroups[i]->setText(tr(client_group_names[i]));
        tempsize = NGroups[i]->sizeHint();
        NGroupsLayout->addWidget(NGroups[i], int(i/3), (i%3));
    }

    NTopFrame = new QFrame(this->centralWidget);
    NTopFrame->setObjectName("NTopFrame");
    NTopFrame->setFrameShape(QFrame::Box);
    NTopFrame->setFrameShadow(QFrame::Sunken);
    theVLayout->addWidget(NTopFrame);


    NCommandList = new QListWidget(this->centralWidget);
    NCommandList->setObjectName("NCommandList");
    NCommandList->adjustSize();
    NCommandList->setGeometry(PADDING, PADDING+120, NCommandList->width(), 0);
    connect(NCommandList, SIGNAL(currentRowChanged(int)), this, SLOT(ChooseCommand()));
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

        NParamFields[i] = new NarsilDoubleEntry(NTopFrame, "NParamLabels");
        connect(dynamic_cast<NarsilDoubleEntry*>(NParamFields[i]),
                SIGNAL(valueChanged(double)),this,SLOT(OmniSync()));

        NParamFields[i]->setFixedWidth(w2/2);
        NParamFields[i]->adjustSize();

        w3 = NParamFields[i]->width();
        h2 = NParamLabels[i]->height();
        h3 = NParamFields[i]->height();

        point.setX(w1 + 2 * PADDING + (i % 2) * (w2 + w3 + PADDING));
        NParamLabels[i]->setGeometry(point.x(), 0, w2, h2);

        point.setX(w1 + PADDING + (i % 2) * (w2 + w3 + PADDING) + w2);
        NParamFields[i]->setGeometry(point.x(), 0, w3, h3);
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
    NAboutLabel->setGeometry(PADDING , PADDING*2+25, 2 * (w2 + w3 + PADDING)
                             + SPACING * 4, tempsize.height());

    h1 = NAboutLabel->height()+PADDING+25;
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

    NSendButton = new QPushButton(NTopFrame);
    NSendButton->setObjectName("NSendButton");
    NSendButton->setText(tr("Send (Shift+F12)"));
    NSendButton->adjustSize();
    NSendButton->setDisabled(true);

    NCurFile = new QLineEdit(NTopFrame);
    NCurFile->setObjectName("NCurFile");
    NCurFile->setText(tr(curfile.toAscii()));
    NCurFile->setReadOnly(true);
    NCurFile->adjustSize();

    NCurFileButton = new QPushButton(NTopFrame);
    NCurFileButton->setText("QCurCaption");
    NCurFileButton->setText(tr("Change"));
    NCurFileButton->adjustSize();

    NVerbose = new QCheckBox(NTopFrame);
    NVerbose->setObjectName("NVerbose");
    NVerbose->setText(tr("Verbose"));
    NVerbose->setChecked(false);
    NVerbose->adjustSize();

    NSendMethod = new QComboBox(NTopFrame);
    NSendMethod->setObjectName("NSendMethod");
    NSendMethod->addItem(tr("LOS"));
    NSendMethod->addItem(tr("TDRSS"));
    NSendMethod->addItem(tr("Iridum"));
    NSendMethod->adjustSize();

    NSendRoute = new QComboBox(NTopFrame);
    NSendRoute->setObjectName("NSendRoute");
    NSendRoute->addItem(tr("COMM 1"));
    NSendRoute->addItem(tr("COMM 2"));
    NSendRoute->adjustSize();

    NCurFile->setGeometry(
                PADDING,
                PADDING + 2 * PADDING + h1 + (int((2 + MAX_N_PARAMS) / 2)) * (h3 + SPACING) - NCurFile->height(),
                NCurFile->width()*1.5,
                NCurFile->height());

    NCurFileButton->setGeometry(
                2*PADDING+NCurFile->width(),
                NCurFile->y(),
                NCurFileButton->width(),
                NCurFile->height());

    NSendMethod->setGeometry(
                3*PADDING+NCurFile->width()+NCurFileButton->width(),
                NCurFile->y(),
                NSendMethod->width(),
                NCurFile->height());

    NSendRoute->setGeometry(
                4*PADDING+NCurFile->width()+NCurFileButton->width()+NSendMethod->width(),
                NCurFile->y(),
                NSendRoute->width(),
                NCurFile->height());

    NVerbose->setGeometry(
                5*PADDING+NCurFile->width()+NCurFileButton->width()+NSendMethod->width()+NSendRoute->width(),
                NCurFile->y(),
                NVerbose->width(),
                NCurFile->height());

    NSendButton->setGeometry(
                2*PADDING + NAboutLabel->width() - NSendButton->width(),
                NCurFile->y(),
                NSendButton->width(),
                NCurFile->height());

    NTopFrame->adjustSize();

    NGroupsBox->adjustSize();
    NGroupsBox->setGeometry(
                2 * PADDING + NCommandList->width(),
                PADDING*2+25,
                NTopFrame->width(),
                NGroupsBox->height()+NOmniBox->height()+PADDING);

    NTopFrame->setGeometry(
                2 * PADDING + NCommandList->width(),
                PADDING * 2 + NGroupsBox->height()+10,
                NTopFrame->width(),
                NTopFrame->height());

    NBotFrame = new QFrame(this->centralWidget);
    NBotFrame->setObjectName("NBotFrame");
    NBotFrame->setFrameShape(QFrame::Box);
    NBotFrame->setFrameShadow(QFrame::Sunken);
    theVLayout->addWidget(NBotFrame);

    NWaitImage = new QLabel(NBotFrame);
    NWaitImage->setText("NWaitImage");
    NWaitImage->setScaledContents(false);
    NWaitImage->setPixmap(*Images[0]);
    NWaitImage->adjustSize();
    NWaitImage->setGeometry(
                NTopFrame->width() - NWaitImage->width() - PADDING,
                PADDING,
                NWaitImage->width(),
                NWaitImage->height());

    NLog = new QTextEdit(NBotFrame);
    NLog->setObjectName("NLog");
    NLog->setReadOnly(true);
    NLog->setGeometry(
                PADDING,
                PADDING,
                NTopFrame->width() - NWaitImage->width() - 3 * PADDING,
                PADDING + NWaitImage->height());

    ReadLog(NLog);

    NBotFrame->adjustSize();
    NBotFrame->setGeometry(
                2*PADDING+NCommandList->width(),
                -25+PADDING * 3 + NGroupsBox->height() + NTopFrame->height(),
                NTopFrame->width(),
                NBotFrame->height());

    NOmniBox->setGeometry(PADDING,PADDING,NCommandList->width()+PADDING+NTopFrame->width(),25);
    NCommandList->setGeometry(PADDING, PADDING+25, NCommandList->width(), PADDING
                              * 2 + NGroupsBox->height() + NTopFrame->height() + NBotFrame->height()-25-25-PADDING);


    if (!curfile.isNull())
        strcpy(_curFileName, curfile.toAscii());
    else
        strcpy(_curFileName, "\0");

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
    connect(NCurFileButton, SIGNAL(clicked()), this, SLOT(ChangeCurFile()));
    connect(timer, SIGNAL(timeout()), this, SLOT(Tick()));
    connect(ping_timer, SIGNAL(timeout()), this, SLOT(Ping()));

    NGroups[0]->setChecked(true);

    NOmniBox->setFocus();
}

MainForm::~MainForm()
{
    Quit();
    delete _dirfile;

    for(int i=0;i<6;i++)
        delete Images[i];

    delete theVLayout;
    delete theHLayout;
    delete centralWidget;
    delete theStatusBar;
}


//***************************************************************************
//****     Main()
//***************************************************************************

Defaults *defaults;
int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    if (argc > 2) {
        printf(
                    "COW " VERSION " doesn't take arguments.  It was compiled at "
                    __DATE__ "\nand is copyright (C) 2002-2011 University of Toronto.\n\n"
                    "This program comes with NO WARRANTY, not even for MERCHANTABILITY or "
                    "FITNESS\n"
                    "FOR A PARTICULAR PURPOSE. You may redistribute it under the terms of "
                    "the GNU\n"
                    "General Public License; see the file named COPYING for details.\n\n"
                    "What was known as Narsil was written by Adam Hincks. It was later\n"
                    "poked at a bit by D.V. Wiebe and then further desecrated by cbn.\n"
                    "Finaly, Joshua Netterfield created COW by fixing the UI, and updating\n"
                    "to Qt4."
                    );
        exit(1);
    }

    /* Host determination */
    if (argc == 2)
    {
        blastcmd_host = argv[1];
    }
    else if ((blastcmd_host = getenv("NARSIL_HOST")) == NULL)
    {
        blastcmd_host = BLASTCMD_HOST;
    }

    /* Client negotiation */
    if(QString(blastcmd_host)=="widow")
    {
        std::cerr<<"Hostname is `widow'. Did you forget to set the NARSIL_HOST variable?\n";
    }
    if (NetCmdConnect(blastcmd_host, 1, 0) < 0) {
        //retry if connection refused (HACK! fixes bug in blastcmd authentication)
        sleep(1);
        printf("Trying to connect one more time\n");
        if (NetCmdConnect(blastcmd_host, 1, 0) < 0)
            exit(16);
    }
    NetCmdGetCmdList();
    NetCmdGetGroupNames();

    defaults = new Defaults();

    MainForm moo(DEF_CURFILE, 0, "moo", 0);
    moo.show();
    return app.exec();
}

// vim: ts=2 sw=2 et
