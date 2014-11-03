/* mcp: the Spider master control program
 *
 * command_struct.h: global definitions for command system
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

#ifndef COMMAND_STRUCT_H
#define COMMAND_STRUCT_H

#include "command_list.h"
#include "channels.h"
#include <time.h>

/* time (in slow frames) to suppress ADC card watchdog, to induce reset */
#define	RESET_ADC_LEN	 80

/* parameters for hk dacs */
#define N_DAC	    32
#define PHASE_MAX   0xffff
#define PHASE_MIN   0
#define DAC_MAX	    0xffff
#define DAC_ZERO    0x8000
#define DAC_MIN	    0

/* number of DAC systems (ie motherboards) */
#define N_DAC_BUS 2


struct Step {
  unsigned short do_step;
  unsigned short start;
  unsigned short end;
  unsigned short nsteps;
  unsigned short which; // only used for bias
  unsigned short dt;
};

struct CommandDataStruct {
  struct {
    unsigned char adc_reset[16];
  } power;

  struct {
    unsigned short bias[N_DAC];
    unsigned char setLevel[N_DAC];
    struct Step step;
  } Bias[N_DAC_BUS];

  struct {
    unsigned short phase[N_DAC];
    struct Step step;
  } Phase[N_DAC_BUS];

  unsigned short df;
  unsigned short bbcFifoSize;

  unsigned short plover;

  struct {
    int cmd_disable;
    int new_cmd;

    enum scanMode {
      AzElNone,
      AzElDisable,
      AzElGoto,
      AzElRaster,
      AzElSet
    } mode;

    double az_accel;
    double el_accel;
    double az_speed;
    double el_speed;
    double az;
    double el;
    double az_ref;
    double el_ref;
    int az_enc_ref;
    int el_enc_ref;

    /* raster-specific: */

    double az_width;
    double el_height;
    
    unsigned int el_Nstep;

  } az_el;

};

void InitCommandData();
int SIndex(enum singleCommand);
int MIndex(enum multiCommand);

extern struct CommandDataStruct CommandData;

#endif   //COMMAND_STRUCT_H

