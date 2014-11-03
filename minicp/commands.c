/* mcp: the Spider master control program
 *
 * commands.c: functions for listening to and processing commands
 *
 * This software is copyright (C) 2002-2010 University of Toronto
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

#include "command_list.h"
#include "command_struct.h"
#include "mcp.h"
#include "tx.h"
#include "channels.h"

/* Seconds since 0TMG jan 1 1970 */
#define SUN_JAN_6_1980 315964800L
/* Seconds in a week */
#define SEC_IN_WEEK  604800L

void nameThread(const char*);  /* mcp.c */

static const char *UnknownCommand = "Unknown Command";

pthread_mutex_t mutex; //init'd in mcp.c

struct CommandDataStruct CommandData;

/** Write the Previous Status: called whenever anything changes */
static void WritePrevStatus()
{
  int fp, n;

  /** write the default file */
  fp = open("/data/etc/minicp/minicp.prev_status", O_WRONLY|O_CREAT|O_TRUNC, 00666);
  if (fp < 0) {
    berror(err, "Commands: mcp.prev_status open()");
    return;
  }

  if ((n = write(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0) {
    berror(err, "Commands: mcp.prev_status write()");
    return;
  }

  if ((n = close(fp)) < 0) {
    berror(err, "Commands: mcp.prev_status close()");
    return;
  }
}

enum singleCommand SCommand(char *cmd)
{
  int i;

  for (i = 0; i < N_SCOMMANDS; i++) {
    if (strcmp(scommands[i].name, cmd) == 0)
      return scommands[i].command;
  }

  return -1;
}

int SIndex(enum singleCommand command)
{
  int i;

  for (i = 0; i < N_SCOMMANDS; i++)
    if (scommands[i].command == command)
      return i;

  return -1;
}

static const char* SName(enum singleCommand command)
{
  int i = SIndex(command);
  return (i == -1) ? UnknownCommand : scommands[i].name;
}

static void SingleCommand (enum singleCommand command, int scheduled)
{
  if (!scheduled)
    bprintf(info, "Commands: Single command: %d (%s)\n", command,
        SName(command));

  /* Update CommandData structure with new info */

  switch (command) {
    case az_el_disable:
      CommandData.az_el.new_cmd = 1;
      CommandData.az_el.mode = AzElDisable;
      break;
    case xyzzy:
      break;
    default:
      bputs(warning, "Commands: ***Invalid Single Word Command***\n");
      return; /* invalid command - no write or update */
  }

  WritePrevStatus();
}

enum multiCommand MCommand(char *cmd)
{
  int i;

  for (i = 0; i < N_MCOMMANDS; i++) {
    if (strcmp(mcommands[i].name, cmd) == 0)
      return mcommands[i].command;
  }

  return -1;
}

int MIndex(enum multiCommand command)
{
  int i;

  for (i = 0; i < N_MCOMMANDS; i++)
    if (mcommands[i].command == command)
      return i;

  return -1;
}

static const char* MName(enum multiCommand command)
{
  int i = MIndex(command);
  return (i == -1) ? UnknownCommand : mcommands[i].name;
}

static void SetParameters(enum multiCommand command, unsigned short *dataq,
    double* rvalues, int* ivalues, char svalues[][CMD_STRING_LEN])
{
  int i, dataqind;
  char type;
  int index = MIndex(command);

  char** dataqc = (char**) dataq;
  /* compute renormalised values - SIPSS FIFO version */
  for (i = dataqind = 0; i < mcommands[index].numparams; ++i) {
    type = mcommands[index].params[i].type;
    if (type == 'i')  /* 15 bit unsigned integer */ {
      ivalues[i] = atoi(dataqc[dataqind++]);
      bprintf(info, "Commands: param%02i: integer: %i\n", i, ivalues[i]);
    } else if (type == 'l')  /* 30 bit unsigned integer */ {
      ivalues[i] = atoi(dataqc[dataqind++]);
      bprintf(info, "Commands: param%02i: long   : %i\n", i, ivalues[i]);
    } else if (type == 'f')  /* 15 bit floating point */ {
      rvalues[i] = atof(dataqc[dataqind++]);
      bprintf(info, "Commands: param%02i: float  : %f\n", i, rvalues[i]);
    } else if (type == 'd') { /* 30 bit floating point */
      rvalues[i] = atof(dataqc[dataqind++]);
      bprintf(info, "Commands: param%02i: double : %f\n", i, rvalues[i]);
    } else if (type == 's') { /* string */
      strncpy(svalues[i], dataqc[dataqind++], CMD_STRING_LEN - 1);
      svalues[i][CMD_STRING_LEN - 1] = 0;
      bprintf(info, "Commands: param%02i: string: %s\n", i, svalues[i]);
    } else
      bprintf(err,
          "Commands: Unknown parameter type ('%c') param%02i: ignored", type,
          i);
  }

  bprintf(info, "Commands: Multiword Command: %d (%s)\n", command,
      MName(command));
}

static inline void copysvalue(char* dest, const char* src)
{
  strncpy(dest, src, CMD_STRING_LEN - 1);
  dest[CMD_STRING_LEN - 1] = '\0';
}

static void MultiCommand(enum multiCommand command, double *rvalues,
    int *ivalues, char svalues[][CMD_STRING_LEN], int scheduled)
{
  int i;
  //char buf[256];

  /* Update CommandData struct with new info
   * If the parameter is type 'i'/'l' set CommandData using ivalues[i]
   * If the parameter is type 'f'/'d' set CommandData using rvalues[i]
   */

  switch(command) {
    case dac1_ampl:
      if (ivalues[0] < N_DAC) {
	CommandData.Bias[0].bias[ivalues[0]] = ivalues[1] << 1;
	CommandData.Bias[0].setLevel[ivalues[0]] = 1;
      } else for (i=0; i<N_DAC; i++) {
	CommandData.Bias[0].bias[i] = ivalues[1] << 1;
	CommandData.Bias[0].setLevel[i] = 1;
      }
      break;
    case dac1_phase:
      if (ivalues[0] < N_DAC) {
	CommandData.Phase[0].phase[ivalues[0]] = rvalues[1] * 65535.0/360.0;
      } else for (i=0; i<N_DAC; i++) {
	CommandData.Phase[0].phase[i] = rvalues[1] * 65535.0/360.0;
      }
      break;
    case bias1_step:
      CommandData.Bias[0].step.do_step = 1;
      CommandData.Bias[0].step.start = ivalues[0] << 1;
      CommandData.Bias[0].step.end = ivalues[1] << 1;
      CommandData.Bias[0].step.nsteps = ivalues[2];
      CommandData.Bias[0].step.dt = ivalues[3];
      CommandData.Bias[0].step.which = ivalues[4];
      break;
    case phase1_step:
      CommandData.Phase[0].step.do_step=1;
      CommandData.Phase[0].step.start=rvalues[0] * 65535.0/360.0;
      CommandData.Phase[0].step.end=rvalues[1] * 65535.0/360.0;
      CommandData.Phase[0].step.nsteps=ivalues[2];
      CommandData.Phase[0].step.dt=ivalues[3];
      break;

    case dac2_ampl:
      if (ivalues[0] < N_DAC) {
	CommandData.Bias[0].bias[ivalues[0]] = ivalues[1] << 1;
	CommandData.Bias[0].setLevel[ivalues[0]] = 1;
      } else for (i=0; i<N_DAC; i++) {
	CommandData.Bias[0].bias[i] = ivalues[1] << 1;
	CommandData.Bias[0].setLevel[i] = 1;
      }
      break;
    case dac2_phase:
      if (ivalues[0] < N_DAC) {
	CommandData.Phase[0].phase[ivalues[0]] = rvalues[1] * 65535.0/360.0;
      } else for (i=0; i<N_DAC; i++) {
	CommandData.Phase[0].phase[i] = rvalues[1] * 65535.0/360.0;
      }
      break;
    case bias2_step:
      CommandData.Bias[0].step.do_step = 1;
      CommandData.Bias[0].step.start = ivalues[0] << 1;
      CommandData.Bias[0].step.end = ivalues[1] << 1;
      CommandData.Bias[0].step.nsteps = ivalues[2];
      CommandData.Bias[0].step.dt = ivalues[3];
      CommandData.Bias[0].step.which = ivalues[4];
      break;
    case phase2_step:
      CommandData.Phase[0].step.do_step=1;
      CommandData.Phase[0].step.start=rvalues[0] * 65535.0/360.0;
      CommandData.Phase[0].step.end=rvalues[1] * 65535.0/360.0;
      CommandData.Phase[0].step.nsteps=ivalues[2];
      CommandData.Phase[0].step.dt=ivalues[3];
      break;

    case reset_adc:
      if (ivalues[0] < 64)
        CommandData.power.adc_reset[ivalues[0]/4] = RESET_ADC_LEN;
      break;
    case az_el_goto:
      CommandData.az_el.az_accel = rvalues[0];
      CommandData.az_el.el_accel = rvalues[1];
      CommandData.az_el.az_speed = rvalues[2];
      CommandData.az_el.el_speed = rvalues[3];
      CommandData.az_el.az = rvalues[4];
      CommandData.az_el.el = rvalues[5];
      CommandData.az_el.new_cmd = 1;
      CommandData.az_el.mode = AzElGoto;
      break;
    case az_el_raster:
      CommandData.az_el.az = rvalues[0];
      CommandData.az_el.el = rvalues[1];
      CommandData.az_el.az_width = rvalues[2];
      CommandData.az_el.el_height = rvalues[3];
      CommandData.az_el.az_speed = rvalues[4];
      CommandData.az_el.el_speed = rvalues[5];
      CommandData.az_el.az_accel = rvalues[6];
      CommandData.az_el.el_accel = rvalues[7];
      CommandData.az_el.el_Nstep = ivalues[8];
      CommandData.az_el.new_cmd = 1;
      CommandData.az_el.mode = AzElRaster;
      break;
    case az_el_set:
      CommandData.az_el.az_ref = rvalues[0];
      CommandData.az_el.el_ref = rvalues[1];
      CommandData.az_el.cmd_disable = 0;
      CommandData.az_el.new_cmd = 1;
      CommandData.az_el.mode = AzElSet;
      break;
    case plugh:/* A hollow voice says "Plugh". */
      CommandData.plover = ivalues[0];
      break;
    default:
      bputs(warning, "Commands: ***Invalid Multi Word Command***\n");
      return; /* invalid command - don't update */
  }

  WritePrevStatus();
}

const char* CommandName(int is_multi, int command)
{
  return (is_multi) ? MName(command) : SName(command);
}

void WatchFIFO ()
{
  unsigned char buf[1];
  char command[100];
  char pbuf[30];
  int fifo;

  int mcommand = -1;
  int mcommand_count = 0;
  char *mcommand_data[DATA_Q_SIZE];

  int i;
  nameThread("SIPSS");
  for (i = 0; i < DATA_Q_SIZE; ++i) {
    mcommand_data[i] = NULL;
  }

  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];
  char svalues[MAX_N_PARAMS][CMD_STRING_LEN];

  int index, pindex = 0;

  bputs(startup, "WatchFIFO startup\n");

  if ((fifo = open("/data/etc/SIPSS.FIFO", O_RDONLY | O_NONBLOCK)) == -1)
    berror(tfatal, "Unable to open FIFO");

  for (;;) {
    index = 0;
    do {
      /* Loop until data come in */
      while (read(fifo, buf, 1) <= 0)
        usleep(10000); /* sleep for 10ms */
      command[index++] = buf[0];
    } while (buf[0] != '\n');
    command[index - 1] = command[index] = 0;
    bprintf(info, "Command received: %s\n", command);
    index = -1;
    while((command[++index] != ' ') && command[index]);
    command[index++] = 0;

    pindex = 0;
    mcommand_count = 0;
    do {
      if ((command[index] == ' ' || command[index] == 0) && pindex > 0) {
        pbuf[pindex] = 0;
        mcommand_data[mcommand_count] =
          reballoc(tfatal, mcommand_data[mcommand_count], pindex + 2);

        strncpy(mcommand_data[mcommand_count++], pbuf, pindex + 1);
        pindex = 0;
      } else {
        pbuf[pindex++] = command[index];
      }
    } while (command[index++] != 0);
    bprintf(info, "%i parameters found.\n", mcommand_count);

    pthread_mutex_lock(&mutex);

    /* Process data */
    if (mcommand_count == 0) {
      mcommand = SCommand(command);
      SingleCommand(mcommand, 0);
      mcommand = -1;
    } else {
      mcommand = MCommand(command);
      bputs(info, "Multi word command received\n");
      if (mcommand_count == mcommands[MIndex(mcommand)].numparams) {
        SetParameters(mcommand, (unsigned short*)mcommand_data, rvalues, ivalues,
            svalues);
        MultiCommand(mcommand, rvalues, ivalues, svalues, 0);
      } else {
        bputs(warning, "Ignoring mal-formed command!\n");
      }
      mcommand = -1;
    }

    /* Relinquish control of memory */
    pthread_mutex_unlock(&mutex);

  }
}

/************************************************************/
/*                                                          */
/*  Initialize CommandData: read last valid state: if there */
/*   is no previous state file, set to default              */
/*                                                          */
/************************************************************/
void InitCommandData()
{
  int fp, n_read = 0, junk, extra = 0, i, j;

  if ((fp = open("/data/etc/minicp/minicp.prev_status", O_RDONLY)) < 0) {
    berror(err, "Commands: Unable to open prev_status file for reading");
  } else {
    if ((n_read = read(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0)
      berror(err, "Commands: prev_status read()");
    if ((extra = read(fp, &junk, sizeof(junk))) < 0)
      berror(err, "Commands: extra prev_status read()");
    if (close(fp) < 0)
      berror(err, "Commands: prev_status close()");
  }

  /** stuff here overrides prev_status **/
  for (i=0; i<16; i++) CommandData.power.adc_reset[i] = 0;

  for (j=0; j<N_DAC_BUS; j++) {
    CommandData.Bias[j].step.do_step = 0;
    CommandData.Phase[j].step.do_step = 0;
    //forces reload of saved bias values
    for (i=0; i<N_DAC; i++) CommandData.Bias[j].setLevel[i] = 1;
  }

  CommandData.az_el.cmd_disable = 1;
  CommandData.az_el.new_cmd = 0;
  CommandData.az_el.mode = AzElNone;

  /** return if we succsesfully read the previous status **/
  if (n_read != sizeof(struct CommandDataStruct))
    bprintf(warning, "Commands: prev_status: Wanted %lu bytes but got %i.\n",
        sizeof(struct CommandDataStruct), n_read);
  else if (extra > 0)
    bputs(warning, "Commands: prev_status: Extra bytes found.\n");
  else
    return;

  bputs(warning, "Commands: Regenerating Command Data and prev_status\n");

  /** prev_status overrides this stuff **/
  for (j=0; j<N_DAC_BUS; j++) {
    for (i=0; i<N_DAC; i++) CommandData.Bias[j].bias[i] = DAC_ZERO;
    CommandData.Bias[j].step.start = DAC_MIN;
    CommandData.Bias[j].step.end = DAC_MAX;
    CommandData.Bias[j].step.nsteps = 1000;
    CommandData.Bias[j].step.dt = 1000;
    CommandData.Bias[j].step.which = 0;

    for (i=0; i<N_DAC; i++) CommandData.Phase[j].phase[i] = PHASE_MIN;
    CommandData.Phase[j].step.start = PHASE_MIN;
    CommandData.Phase[j].step.end = PHASE_MAX;
    CommandData.Phase[j].step.nsteps = 1000;
    CommandData.Phase[j].step.dt = 1000;
  }

  CommandData.az_el.az_accel = 1.0;
  CommandData.az_el.el_accel = 1.0;
  CommandData.az_el.az_speed = 1.0;
  CommandData.az_el.el_speed = 1.0;
  CommandData.az_el.az = 0.0;
  CommandData.az_el.el = 0.0;
  CommandData.az_el.az_width = 30.0;
  CommandData.az_el.el_Nstep = 10; 
  CommandData.az_el.el_height = 10.0;
  CommandData.az_el.az_ref = 0.0;
  CommandData.az_el.el_ref = 0.0;
  CommandData.az_el.az_enc_ref = 0;
  CommandData.az_el.el_enc_ref = 0; 
 
  CommandData.plover = 0;

  WritePrevStatus();
}
