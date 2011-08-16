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
#include "share/lut.h"
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
  static struct NiosStruct* NiosAddr[N_DAC];
  static struct NiosStruct* phaseStepEnaAddr;
  static struct NiosStruct* phaseStepStartAddr;
  static struct NiosStruct* phaseStepEndAddr;
  static struct NiosStruct* phaseStepNstepsAddr;
  static struct NiosStruct* phaseStepTimeAddr;

  char field[20];
  int i;
  static int step_size = 1;
  static unsigned int k = 0;
  static unsigned int dk = 1;
  static int end = 0;
  static int start = 0;
  int phase = 0;

  if (first_time) {
    first_time = 0;
    for(i = 0; i < N_DAC; i++) {
      sprintf(field, "phase_%02d_hk", i);
      NiosAddr[i] = GetNiosAddr(field);
    }
    phaseStepEnaAddr	= GetNiosAddr("step_ena_phase");
    phaseStepStartAddr	= GetNiosAddr("step_start_phase");
    phaseStepEndAddr	= GetNiosAddr("step_end_phase");
    phaseStepNstepsAddr	= GetNiosAddr("step_nsteps_phase");
    phaseStepTimeAddr	= GetNiosAddr("step_time_phase");
   
  }	

  if(CommandData.Phase.step.do_step) {
    if (k == 0) {
      start = CommandData.Phase.step.start;
      end = CommandData.Phase.step.end;
      step_size = (end - start)/CommandData.Phase.step.nsteps;

      if(step_size == 0) { // minimum step size is 1
	if (end >= start) step_size = 1;
	if (end < start) step_size = -1;
      }
      end += step_size;
      dk = CommandData.Phase.step.dt*SR/1000/FAST_PER_SLOW;
      if (dk == 0) dk = 1;
    }

    phase = start + (k++/dk)*step_size;

    if (step_size > 0) {
      if (phase > PHASE_MAX) phase = PHASE_MAX;
      if (phase >= end || phase == PHASE_MAX) CommandData.Phase.step.do_step=0;
    } else {
      if (phase < PHASE_MIN) phase = PHASE_MIN;
      if (phase <= end || phase == PHASE_MIN) CommandData.Phase.step.do_step=0; 
    }

    WriteData(phaseStepEnaAddr,CommandData.Phase.step.do_step, NIOS_QUEUE);
    WriteCalData(phaseStepStartAddr,CommandData.Phase.step.start, NIOS_QUEUE);
    WriteCalData(phaseStepEndAddr,CommandData.Phase.step.end, NIOS_QUEUE);
    WriteData(phaseStepNstepsAddr,CommandData.Phase.step.nsteps, NIOS_QUEUE);
    WriteData(phaseStepTimeAddr,CommandData.Phase.step.dt, NIOS_QUEUE);
    for(i = 0; i < N_DAC; i++) WriteData(NiosAddr[i], phase, NIOS_QUEUE);
  } else {
    for(i = 0; i < N_DAC; i++)
      WriteData(NiosAddr[i], CommandData.Phase.phase[i], NIOS_QUEUE);
    k = 0;
  }
}

/************************************************************************/
/*                                                                      */
/*   BiasControl: Amplitude of HK DAC signals (bias and heat)           */
/*                                                                      */
/************************************************************************/
void BiasControl()
{
  static struct NiosStruct* amplBiasAddr[N_DAC];
  static struct NiosStruct* stepEnaBiasAddr;
  static struct NiosStruct* stepStartBiasAddr;
  static struct NiosStruct* stepEndBiasAddr;
  static struct NiosStruct* stepNBiasAddr;
  static struct NiosStruct* stepTimeBiasAddr;
  static struct NiosStruct* stepWhichBiasAddr;
  char field[20];
  static unsigned int step_size = 1;
  static unsigned int k = 0;
  static unsigned int dk = 1;
  static int end = 0;
  static int start = 0;
  int bias = 0;

  int i;

  /******** Obtain correct indexes the first time here ***********/
  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    for (i=0; i<N_DAC; i++) {
      sprintf(field, "ampl_%02d_hk", i);
      amplBiasAddr[i] = GetNiosAddr(field);
    }
    stepEnaBiasAddr   = GetNiosAddr("step_ena_bias");
    stepStartBiasAddr = GetNiosAddr("step_start_bias");
    stepEndBiasAddr   = GetNiosAddr("step_end_bias");
    stepNBiasAddr     = GetNiosAddr("step_n_bias");
    stepTimeBiasAddr  = GetNiosAddr("step_time_bias");
    stepWhichBiasAddr = GetNiosAddr("step_which_bias");
  }

  /************* Set the Bias Levels *******/
  //TODO should DAC levels use the same scale for both AC and DC?
  for (i=0; i<N_DAC; i++) {
    if (CommandData.Bias.setLevel[i]) {
      WriteData(amplBiasAddr[i], CommandData.Bias.bias[i], NIOS_QUEUE);
      CommandData.Bias.setLevel[i] = 0;
      CommandData.Bias.step.do_step = 0;
    }
  }

  if (CommandData.Bias.step.do_step) {
    if (k == 0) {
      start = CommandData.Bias.step.start;
      end = CommandData.Bias.step.end;
      step_size = (end - start)/CommandData.Bias.step.nsteps;

      if(step_size == 0) { // minimum step size is 1
	if (end >= start) step_size = 1;
	if (end < start) step_size = -1;
      }
      end +=step_size;
      dk = CommandData.Bias.step.dt*SR/1000/FAST_PER_SLOW;
      if (dk == 0) dk = 1; 
    }

    bias = start + (k++/dk)*step_size;

    if (step_size > 0) {
      if (bias > DAC_MAX) bias = DAC_MAX;
      if (bias >= end || bias == DAC_MAX) CommandData.Bias.step.do_step=0;
    } else {
      if (bias < DAC_MIN) bias = DAC_MIN;
      if (bias <= end || bias == DAC_MIN) CommandData.Bias.step.do_step=0; 
    }

    WriteData(stepEnaBiasAddr,CommandData.Bias.step.do_step, NIOS_QUEUE);
    WriteData(stepStartBiasAddr,CommandData.Bias.step.start, NIOS_QUEUE);
    WriteData(stepEndBiasAddr,CommandData.Bias.step.end, NIOS_QUEUE);
    WriteData(stepNBiasAddr,CommandData.Bias.step.nsteps, NIOS_QUEUE);
    WriteData(stepTimeBiasAddr,CommandData.Bias.step.dt, NIOS_QUEUE);
    WriteData(stepWhichBiasAddr,CommandData.Bias.step.which, NIOS_QUEUE);
    if (CommandData.Bias.step.which < N_DAC) {  //just one
      WriteData(amplBiasAddr[CommandData.Bias.step.which], bias, NIOS_QUEUE);
    } else {					//all
      for(i = 0; i <= N_DAC; i++)
	WriteData(amplBiasAddr[i], bias, NIOS_QUEUE);
    }
  } else k = 0;
}
