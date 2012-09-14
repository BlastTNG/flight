/* pcm: the Spider master control program
 *
 * hk.c: handle housekeeping commands and logic
 *
 * This software is copyright (C) 2011 University of Toronto
 *
 * This file is part of pcm.
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

#include <math.h>
#include <stdio.h>

#include "mcp.h"
//#include "share/lut.h"    //TODO pcm will need LUTs for HK cal
#include "tx.h"
#include "command_struct.h"

/************************************************************************/
/*                                                                      */
/* PhaseControl: set phase shifts for the bias channels                 */
/*                                                                      */
/************************************************************************/
void PhaseControl()
{
  static int first_time = 1;
  static struct NiosStruct* phaseNiosAddr[N_DAC_BUS][N_DAC];
  static struct NiosStruct* phaseStepEnaAddr[N_DAC_BUS];
  static struct NiosStruct* phaseStepStartAddr[N_DAC_BUS];
  static struct NiosStruct* phaseStepEndAddr[N_DAC_BUS];
  static struct NiosStruct* phaseStepNstepsAddr[N_DAC_BUS];
  static struct NiosStruct* phaseStepTimeAddr[N_DAC_BUS];

  char field[20];
  int i, j;
  static int step_size = 1;
  static unsigned int k = 0;
  static unsigned int dk = 1;
  static int end = 0;
  static int start = 0;
  int phase = 0;

  if (first_time) {
    first_time = 0;
    for (j = 0; j < N_DAC_BUS; j++) {
      for(i = 0; i < N_DAC; i++) {
        sprintf(field, "dac%1d%02d_phase", j+1, i);
        phaseNiosAddr[j][i] = GetNiosAddr(field);
      }
      sprintf(field, "step%1d_ena_phase", j+1);
      phaseStepEnaAddr[j] = GetNiosAddr(field);
      sprintf(field, "step%1d_start_phase", j+1);
      phaseStepStartAddr[j] = GetNiosAddr(field);
      sprintf(field, "step%1d_end_phase", j+1);
      phaseStepEndAddr[j] = GetNiosAddr(field);
      sprintf(field, "step%1d_nsteps_phase", j+1);
      phaseStepNstepsAddr[j] = GetNiosAddr(field);
      sprintf(field, "step%1d_time_phase", j+1);
      phaseStepTimeAddr[j] = GetNiosAddr(field);

    }	
  }

  for (j = 0; j < N_DAC_BUS; j++) {
    if(CommandData.Phase[j].step.do_step) {
      if (k == 0) {
        start = CommandData.Phase[j].step.start;
        end = CommandData.Phase[j].step.end;
        step_size = (end - start)/CommandData.Phase[j].step.nsteps;

        if(step_size == 0) { // minimum step size is 1
          if (end >= start) step_size = 1;
          if (end < start) step_size = -1;
        }
        end += step_size;
        dk = CommandData.Phase[j].step.dt*SR/1000/FAST_PER_SLOW;
        if (dk == 0) dk = 1;
      }

      phase = start + (k++/dk)*step_size;

      if (step_size > 0) {
        if (phase > PHASE_MAX) phase = PHASE_MAX;
        if (phase >= end || phase == PHASE_MAX)
          CommandData.Phase[j].step.do_step=0;
      } else {
        if (phase < PHASE_MIN) phase = PHASE_MIN;
        if (phase <= end || phase == PHASE_MIN)
          CommandData.Phase[j].step.do_step=0; 
      }

      WriteData(phaseStepEnaAddr[j],
          CommandData.Phase[j].step.do_step, NIOS_QUEUE);
      WriteCalData(phaseStepStartAddr[j],
          CommandData.Phase[j].step.start, NIOS_QUEUE);
      WriteCalData(phaseStepEndAddr[j],
          CommandData.Phase[j].step.end, NIOS_QUEUE);
      WriteData(phaseStepNstepsAddr[j],
          CommandData.Phase[j].step.nsteps, NIOS_QUEUE);
      WriteData(phaseStepTimeAddr[j],
          CommandData.Phase[j].step.dt, NIOS_QUEUE);
      for(i = 0; i < N_DAC; i++)
        WriteData(phaseNiosAddr[j][i], phase, NIOS_QUEUE);
    } else {
      for(i = 0; i < N_DAC; i++)
        WriteData(phaseNiosAddr[j][i],
            CommandData.Phase[j].phase[i], NIOS_QUEUE);
      k = 0;
    }
  }
}

/************************************************************************/
/*                                                                      */
/*   BiasControl: Amplitude of HK DAC signals (bias and heat)           */
/*                                                                      */
/************************************************************************/
void BiasControl()
{
  static struct NiosStruct* amplBiasAddr[N_DAC_BUS][N_DAC];
  static struct NiosStruct* stepEnaBiasAddr[N_DAC_BUS];
  static struct NiosStruct* stepStartBiasAddr[N_DAC_BUS];
  static struct NiosStruct* stepEndBiasAddr[N_DAC_BUS];
  static struct NiosStruct* stepNBiasAddr[N_DAC_BUS];
  static struct NiosStruct* stepTimeBiasAddr[N_DAC_BUS];
  static struct NiosStruct* stepWhichBiasAddr[N_DAC_BUS];
  char field[20];
  static unsigned int step_size = 1;
  static unsigned int k = 0;
  static unsigned int dk = 1;
  static int end = 0;
  static int start = 0;
  int bias = 0;

  int i, j;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    for (j=0; j<N_DAC_BUS; j++) {
      for (i=0; i<N_DAC; i++) {
        sprintf(field, "dac%1d%02d_ampl", j+1, i);
        amplBiasAddr[j][i] = GetNiosAddr(field);
      }
      sprintf(field, "step%1d_ena_bias", j+1);
      stepEnaBiasAddr[j] = GetNiosAddr(field);
      sprintf(field, "step%1d_start_bias", j+1);
      stepStartBiasAddr[j] = GetNiosAddr(field);
      sprintf(field, "step%1d_end_bias", j+1);
      stepEndBiasAddr[j] = GetNiosAddr(field);
      sprintf(field, "step%1d_n_bias", j+1);
      stepNBiasAddr[j] = GetNiosAddr(field);
      sprintf(field, "step%1d_time_bias", j+1);
      stepTimeBiasAddr[j] = GetNiosAddr(field);
      sprintf(field, "step%1d_which_bias", j+1);
      stepWhichBiasAddr[j] = GetNiosAddr(field);
    }
  }

  for (j=0; j<N_DAC_BUS; j++) {
    /************* Set the Bias Levels *******/
    //TODO should DAC levels use the same scale for both AC and DC?
    for (i=0; i<N_DAC; i++) {
      if (CommandData.Bias[j].setLevel[i]) {
        WriteData(amplBiasAddr[j][i], CommandData.Bias[j].bias[i], NIOS_QUEUE);
        CommandData.Bias[j].setLevel[i] = 0;
        CommandData.Bias[j].step.do_step = 0;
      }
    }

    if (CommandData.Bias[j].step.do_step) {
      if (k == 0) {
        start = CommandData.Bias[j].step.start;
        end = CommandData.Bias[j].step.end;
        step_size = (end - start)/CommandData.Bias[j].step.nsteps;

        if(step_size == 0) { // minimum step size is 1
          if (end >= start) step_size = 1;
          if (end < start) step_size = -1;
        }
        end +=step_size;
        dk = CommandData.Bias[j].step.dt*SR/1000/FAST_PER_SLOW;
        if (dk == 0) dk = 1; 
      }

      bias = start + (k++/dk)*step_size;

      if (step_size > 0) {
        if (bias > DAC_MAX) bias = DAC_MAX;
        if (bias >= end || bias == DAC_MAX) CommandData.Bias[j].step.do_step=0;
      } else {
        if (bias < DAC_MIN) bias = DAC_MIN;
        if (bias <= end || bias == DAC_MIN) CommandData.Bias[j].step.do_step=0; 
      }

      WriteData(stepEnaBiasAddr[j],
          CommandData.Bias[j].step.do_step, NIOS_QUEUE);
      WriteData(stepStartBiasAddr[j],
          CommandData.Bias[j].step.start, NIOS_QUEUE);
      WriteData(stepEndBiasAddr[j],
          CommandData.Bias[j].step.end, NIOS_QUEUE);
      WriteData(stepNBiasAddr[j],
          CommandData.Bias[j].step.nsteps, NIOS_QUEUE);
      WriteData(stepTimeBiasAddr[j],
          CommandData.Bias[j].step.dt, NIOS_QUEUE);
      WriteData(stepWhichBiasAddr[j],
          CommandData.Bias[j].step.which, NIOS_QUEUE);
      if (CommandData.Bias[j].step.which < N_DAC) {  //just one
        WriteData(amplBiasAddr[j][CommandData.Bias[j].step.which],
          bias, NIOS_QUEUE);
      } else {					//all
        for(i = 0; i <= N_DAC; i++)
          WriteData(amplBiasAddr[j][i], bias, NIOS_QUEUE);
      }
    } else k = 0;
  }
}
