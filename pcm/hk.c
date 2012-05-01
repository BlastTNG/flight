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
#include "lut.h"
#include "tx.h"
#include "command_struct.h"

/************************************************************************/
/*                                                                      */
/* PhaseControl: set phase shifts for the bias channels                 */
/*                                                                      */
/************************************************************************/
static void PhaseControl()
{
  static struct NiosStruct* phaseCnxAddr[6];
  static struct NiosStruct* phaseNtdAddr[6];
  char field[20];
  int i;

  static int first_time = 1;
  if (first_time) {
    first_time = 0;
    for(i = 0; i < 6; i++) {
      sprintf(field, "ph_cnx_%1d_hk", i+1);
      phaseCnxAddr[i] = GetNiosAddr(field);
      sprintf(field, "ph_ntd_%1d_hk", i+1);
      phaseNtdAddr[i] = GetNiosAddr(field);
    }
  }	

  for(i = 0; i < 6; i++) {
    WriteCalData(phaseCnxAddr[i], CommandData.hk[i].cernox.phase, NIOS_QUEUE);
    WriteCalData(phaseNtdAddr[i], CommandData.hk[i].ntd.phase, NIOS_QUEUE);
  }
}

/************************************************************************/
/*                                                                      */
/*   BiasControl: Amplitude of HK DAC signals (bias and heat)           */
/*                                                                      */
/************************************************************************/
static void BiasControl()
{
  static struct NiosStruct* vCnxAddr[6];
  static struct NiosStruct* vNtdAddr[6];
  static struct NiosStruct* fBiasCmdHkAddr;
  static struct NiosStruct* fBiasHkAddr;
  char field[20];
  int i;
  unsigned short period;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    for (i=0; i<6; i++) {
      sprintf(field, "v_cnx_%1d_hk", i+1);
      vCnxAddr[i] = GetNiosAddr(field);
      sprintf(field, "v_ntd_%1d_hk", i+1);
      vNtdAddr[i] = GetNiosAddr(field);
    }
    fBiasCmdHkAddr = GetNiosAddr("f_bias_cmd_hk");
    fBiasHkAddr = GetNiosAddr("f_bias_hk");
  }

  //otherwise need to change scaling in tx_struct
  for (i=0; i<6; i++) {
    WriteCalData(vCnxAddr[i], CommandData.hk[i].cernox.ampl, NIOS_QUEUE);
    WriteCalData(vNtdAddr[i], CommandData.hk[i].ntd.ampl, NIOS_QUEUE);
  }
  WriteData(fBiasCmdHkAddr, CommandData.hk_bias_freq, NIOS_QUEUE);
  period = ADC_SR/CommandData.hk_bias_freq; //cast as short important here!
  WriteCalData(fBiasHkAddr, ADC_SR/period, NIOS_QUEUE);
}

/************************************************************************/
/*                                                                      */
/*   HeatControl: Switching logic for the PWM (digital) heaters         */
/*                                                                      */
/************************************************************************/
/* bit positions of hk pwm heaters */
#define HK_PWM_PUMP   0x01
#define HK_PWM_HSW    0x02
#define HK_PWM_TILE3  0x04
#define HK_PWM_TILE2  0x08
#define HK_PWM_TILE1  0x10
#define HK_PWM_FPHI   0x20
#define HK_PWM_TILE4  0x40

static void HeatControl()
{
  static struct NiosStruct* heat13Addr;
  static struct NiosStruct* heat45Addr;
  static struct NiosStruct* heat26Addr;
  static struct NiosStruct* heatTAddr;
  static struct NiosStruct* heatSsaAddr[6];
  static struct NiosStruct* heatFploAddr[6];

  int i;
  unsigned short temp;
  unsigned short bits[6];
  char buf[16];

  static int first_time = 1;
  if (first_time) {
    first_time = 0;
    heat13Addr = GetNiosAddr("heat_13_hk");
    heat45Addr = GetNiosAddr("heat_45_hk");
    heat26Addr = GetNiosAddr("heat_26_hk");
    heatTAddr = GetNiosAddr("heat_t_hk");
    for (i=0; i<6; i++) {
      sprintf(buf, "heat_ssa_%1d_hk", i+1);
      heatSsaAddr[i] = GetNiosAddr(buf);
    }
    for (i=0; i<6; i++) {
      sprintf(buf, "heat_fplo_%1d_hk", i+1);
      heatFploAddr[i] = GetNiosAddr(buf);
    }
  }	

  //PWM heaters
  for (i=0; i<6; i++) {
    bits[i] = 0;
    if (CommandData.hk[i].pump_heat) bits[i] |= HK_PWM_PUMP;
    //NB: heat switch is normally closed, so logic inverted
    if (!CommandData.hk[i].heat_switch) bits[i] |= HK_PWM_HSW;
    if (CommandData.hk[i].fphi_heat) bits[i] |= HK_PWM_FPHI;
    if (CommandData.hk[i].tile_heat[0]) bits[i] |= HK_PWM_TILE1;
    if (CommandData.hk[i].tile_heat[0] > 0) CommandData.hk[i].tile_heat[0]--;
    if (CommandData.hk[i].tile_heat[1]) bits[i] |= HK_PWM_TILE2;
    if (CommandData.hk[i].tile_heat[1] > 0) CommandData.hk[i].tile_heat[1]--;
    if (CommandData.hk[i].tile_heat[2]) bits[i] |= HK_PWM_TILE3;
    if (CommandData.hk[i].tile_heat[2] > 0) CommandData.hk[i].tile_heat[2]--;
    if (CommandData.hk[i].tile_heat[3]) bits[i] |= HK_PWM_TILE4;
    if (CommandData.hk[i].tile_heat[3] > 0) CommandData.hk[i].tile_heat[3]--;
  }
  temp = ((bits[0] & 0xff) << 8) | (bits[2] & 0xff);
  WriteData(heat13Addr, temp, NIOS_QUEUE);
  temp = ((bits[3] & 0xff) << 8) | (bits[4] & 0xff);
  WriteData(heat45Addr, temp, NIOS_QUEUE);
  temp = ((bits[1] & 0xff) << 8) | (bits[5] & 0xff);
  WriteData(heat26Addr, temp, NIOS_QUEUE);

  WriteData(heatTAddr, CommandData.hk_theo_heat&0x00ff, NIOS_QUEUE);

  //DAC heaters
  for (i=0; i<6; i++) {
    WriteCalData(heatSsaAddr[i], CommandData.hk[i].ssa_heat, NIOS_QUEUE);
    WriteCalData(heatFploAddr[i], CommandData.hk[i].fplo_heat, NIOS_QUEUE);
  }
}

void HouseKeeping()
{
  static struct NiosStruct* insertLastHkAddr;
  static struct NiosStruct* tileLastHkAddr;
  static struct NiosStruct* pulseLastHkAddr;
  static struct NiosStruct* vHeatLastHkAddr;
  static int first_time = 1;
  if (first_time) {
    first_time = 0;
    insertLastHkAddr = GetNiosAddr("insert_last_hk");
    tileLastHkAddr = GetNiosAddr("tile_last_hk");
    pulseLastHkAddr = GetNiosAddr("pulse_last_hk");
    vHeatLastHkAddr = GetNiosAddr("v_heat_last_hk");
  }

  BiasControl();
  PhaseControl();
  HeatControl();

  WriteData(insertLastHkAddr, CommandData.hk_last, NIOS_QUEUE);
  WriteData(tileLastHkAddr, CommandData.hk_tile_last, NIOS_QUEUE);
  WriteData(pulseLastHkAddr, CommandData.hk_pulse_last, NIOS_QUEUE);
  WriteCalData(vHeatLastHkAddr, CommandData.hk_vheat_last, NIOS_QUEUE);
}
