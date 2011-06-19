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

struct CommandDataStruct {
  struct {
    unsigned char adc_reset[16];
  } power;
  int lockin_phase[4];       //phase of locking wave in 2000ths of a cycle
  int bias_ampl;             //bias amplitude in counts (full scale = 32767)

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

    /* raster-specific: */

    double az_width;
    double el_step;
    double el_height;

  } az_el;

};

void InitCommandData();
int SIndex(enum singleCommand);
int MIndex(enum multiCommand);

extern struct CommandDataStruct CommandData;

#endif   //COMMAND_STRUCT_H

