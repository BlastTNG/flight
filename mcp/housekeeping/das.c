/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 * And copyright 2010 Matthew Truch
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

#include <math.h>
#include <stdio.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"

/* the HWPR pot fraction of full scale should not read higher than this */
#define HWPR_POT_MAX	0.99

/* Heater control bits (BIAS_D G4) */
#define HEAT_HELIUM_LEVEL    0x01
#define HEAT_CHARCOAL        0x02
#define HEAT_POT_HS          0x04
#define HEAT_CHARCOAL_HS     0x08
#define HEAT_JFET            0x10
#define HEAT_BDA             0x20
#define HEAT_CALIBRATOR      0x40
#define HEAT_HWPR_POS        0x80

/* Valve control bits (BIAS_D G3) */
/* NB: VALVE_*_ON bits are all active low in hardware, but inverted in the DSP
 * so that the default DSP state (0x0000) leaves valves off */
#define VALVE_LHe_ON      0x01
#define VALVE_L_OPEN      0x04
#define VALVE_L_CLOSE     0x02
#define VALVE_LN_ON       0x08
#define VALVE_POT_ON      0x90	  //more cowbell
#define VALVE_POT_OPEN    0x40
#define VALVE_POT_CLOSE   0x20

/* he3 cycle */
#define CRYO_CYCLE_OUT    0x0000    //out of LHe. Nothing to do
#define CRYO_CYCLE_COLD   0x0001    //Cold. All is well
#define CRYO_CYCLE_ON     0x0002    //Charcoal heating
#define CRYO_CYCLE_COOL   0x0004    //Cycle done, cooling
#define CRYO_CYCLE_HS_OFF 0x0008    //Start of cycle, open head switch
#define CRYO_CYCLE_SETTLE 0x0010    //Charcoal off, settle before connecting HS

/* he3 cycle timeouts (all in seconds) */
#define CRYO_CYCLE_COOL_TIMEOUT  (2*60*60)
#define CRYO_CYCLE_HS_TIMEOUT    (2.75*60)

#define T_LHE_MAX         4.600
#define T_CHAR_HS_COLD   10.000

/* CryoState bitfield */
//#define CS_HELIUMLEVEL    0x0001
//#define CS_CHARCOAL       0x0002
//#define CS_HWPR_POS       0x0004
#define CS_POTVALVE_ON    0x0010
#define CS_POTVALVE_OPEN  0x0020
#define CS_LHeVALVE_ON    0x0040
#define CS_LVALVE_OPEN    0x0080
#define CS_LNVALVE_ON     0x0100
#define CS_AUTO_JFET      0x0200

extern int hwpr_calpulse_flag; // defined in hwpr.c

/***********************************************************************/
/* CalLamp: Flash calibrator                                           */
/***********************************************************************/
static int CalLamp (int index)
{
  static channel_t* pulseCalAddr;
  static int pulse_cnt = 0;             //count of pulse length
  static unsigned int elapsed = 0;  //DEBUG_CAL count for wait between pulses
  static enum calmode last_mode = off;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    pulseCalAddr = channels_find_by_name("pulse_cal");
  }

  /* Save the current pulse length */
  if (index == 0)
    SET_VALUE(pulseCalAddr, CommandData.Cryo.calib_pulse);

  if (CommandData.Cryo.calibrator == on) {
    last_mode = on;
    return 1;


  } else if (CommandData.Cryo.calibrator == repeat) {
    //    CommandData.Cryo.calibrator = off; 
    if (hwpr_calpulse_flag && CommandData.Cryo.calib_hwpr) {
      pulse_cnt = CommandData.Cryo.calib_pulse;
      elapsed=0;
      hwpr_calpulse_flag=0;
      blast_info("CalLamp: Hey! Got a hwpr_calpulse_flag! pulse_cnt = %i, elapsed = %i",pulse_cnt, elapsed);
    } else if ((CommandData.Cryo.calib_period > 0 && elapsed >= CommandData.Cryo.calib_period) || last_mode != repeat) {
      elapsed = 0;
      pulse_cnt = CommandData.Cryo.calib_pulse;
      blast_info("CalLamp: We haven't had a pulse in a while!  Let's send one! pulse_cnt = %i, elapsed = %i",pulse_cnt, elapsed);
    } else if (index == 0) elapsed++;  //period measured in slow frames
    last_mode = repeat;
  } else if (CommandData.Cryo.calibrator == pulse) {
      if (last_mode != pulse) pulse_cnt = CommandData.Cryo.calib_pulse;
      last_mode = pulse;
      if (pulse_cnt <= 0) last_mode = CommandData.Cryo.calibrator = off;
  } else {    //off mode
    pulse_cnt = 0;
    last_mode = off;
  }

  if (pulse_cnt > 0) {
    pulse_cnt--;
    return 1;
  }
  return 0;
}

/* Automatic conrol of JFET heater */
static int JFETthermostat(void)
{
  double jfet_temp;
  static channel_t* tJfetAddr;

  static struct LutType DiodeLut =
     {"/data/etc/blast/dt600.txt", 0, NULL, NULL, 0};

  static int lasttime = 0;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    tJfetAddr = channels_find_by_name("td_jfet");
    LutInit(&DiodeLut);
  }

  jfet_temp = ReadCalData(tJfetAddr);
  jfet_temp = LutCal(&DiodeLut, jfet_temp);

  if (jfet_temp < 0 || jfet_temp > 400)
    return CommandData.Cryo.JFETHeat;
  else if (jfet_temp > CommandData.Cryo.JFETSetOff)
  {
    lasttime = 0;
    return 0;
  }
  else if (jfet_temp < CommandData.Cryo.JFETSetOn)
  {
    lasttime = 1;
    return 1;
  }
  else //We're inbetween the setpoints, so keep doin' what we were doin'
    return lasttime;
}

static void FridgeCycle(int *heatctrl, int *cryostate, int reset, uint16_t  *force_cycle)
{
    static int firsttime = 1000;
    // Skip first 1000 frames (10 seconds) as startup veto.
    static channel_t* t_lhe_Addr;
    static channel_t* t_he3fridge_Addr;
    static channel_t* t_charcoal_Addr;
    static channel_t* t_he4pot_Addr;
    static channel_t* t_char_hs_Addr;
    static channel_t* startCycleAddr;
    static channel_t* stateCycleAddr;
    static channel_t* startSetCycleAddr;

    //for writing fridge cycle parameters back to the frame
    static channel_t* tStartCycleAddr;
    static channel_t* tPotMaxCycleAddr;
    static channel_t* tCharMaxCycleAddr;
    static channel_t* tCharSetCycleAddr;
    static channel_t* timeCharCycleAddr;
    static channel_t* timeSetCycleAddr;

    static struct LutType DiodeLut = { "/data/etc/blast/dt600.txt", 0, NULL, NULL, 0 };
    static struct LutType HSLut = { "/data/etc/blast/dt-simonchase.txt", 0, NULL, NULL, 0 };
    static struct LutType ROXLut = { "/data/etc/blast/rox-raw.txt", 0, NULL, NULL, 0 };

    double t_lhe, t_he3fridge, t_charcoal, t_he4pot, t_char_hs;

    //derived command parameters
    double t_he3fridge_cold;
    time_t overall_settle_timeout;

    time_t start_time, settle_start_time;
    uint16_t  cycle_state;
    static uint16_t  iterator = 1;
    static uint16_t  heat_char = 0;
    static uint16_t  heat_hs = 0;

    if (firsttime > 1) {
        firsttime--;
        return;
    }
    else if (firsttime) {
        firsttime = 0;
        t_lhe_Addr = channels_find_by_name("td_lhe");
        t_he3fridge_Addr = channels_find_by_name("tr_300mk_strap"); //NOTE: tr_he3fridge is broken.
        t_charcoal_Addr = channels_find_by_name("td_charcoal");
        t_he4pot_Addr = channels_find_by_name("tr_m4"); //NOTE: tr_he4_pot is broken.
        t_char_hs_Addr = channels_find_by_name("td_hs_charcoal");
        startCycleAddr = channels_find_by_name("start_cycle");
        stateCycleAddr = channels_find_by_name("state_cycle");
        startSetCycleAddr = channels_find_by_name("start_set_cycle");
        SET_VALUE(stateCycleAddr, CRYO_CYCLE_OUT);

        //for writing fridge cycle parameters back to the frame
        tStartCycleAddr = channels_find_by_name("t_start_cycle");
        tPotMaxCycleAddr = channels_find_by_name("t_pot_max_cycle");
        tCharMaxCycleAddr = channels_find_by_name("t_char_max_cycle");
        tCharSetCycleAddr = channels_find_by_name("t_char_set_cycle");
        timeCharCycleAddr = channels_find_by_name("time_char_cycle");
        timeSetCycleAddr = channels_find_by_name("time_set_cycle");

        LutInit(&DiodeLut);
        LutInit(&HSLut);
        LutInit(&ROXLut);
    }

    SET_SCALED_VALUE(tStartCycleAddr, CommandData.Cryo.cycle_start_temp);
    SET_SCALED_VALUE(tPotMaxCycleAddr, CommandData.Cryo.cycle_pot_max);
    SET_SCALED_VALUE(tCharMaxCycleAddr, CommandData.Cryo.cycle_charcoal_max);
    SET_SCALED_VALUE(tCharSetCycleAddr, CommandData.Cryo.cycle_charcoal_settle);
    SET_SCALED_VALUE(timeCharCycleAddr, CommandData.Cryo.cycle_charcoal_timeout);
    SET_SCALED_VALUE(timeSetCycleAddr, CommandData.Cryo.cycle_settle_timeout);

    overall_settle_timeout = CommandData.Cryo.cycle_charcoal_timeout + CommandData.Cryo.cycle_settle_timeout + 2;
    t_he3fridge_cold = CommandData.Cryo.cycle_start_temp - 0.025;

    if (reset || force_cycle == NULL) {
        SET_VALUE(stateCycleAddr, CRYO_CYCLE_OUT);
        iterator = 1;
        heat_char = 0;
        heat_hs = 0;
        return;
    }

    if (!(iterator++ % 199)) /* Run this loop at ~0.5 Hz */
    {
        GET_VALUE(startCycleAddr, start_time);
        GET_VALUE(stateCycleAddr, cycle_state);
        GET_VALUE(startSetCycleAddr, settle_start_time);

        /* Extract and calculate all the temperatures of interest */
        t_lhe = ReadCalData(t_lhe_Addr);
        t_charcoal = ReadCalData(t_charcoal_Addr);
        t_he3fridge = ReadCalData(t_he3fridge_Addr);
        t_he4pot = ReadCalData(t_he4pot_Addr);
        t_char_hs = ReadCalData(t_char_hs_Addr);

        t_lhe = LutCal(&DiodeLut, t_lhe);
        t_charcoal = LutCal(&DiodeLut, t_charcoal);
        t_char_hs = LutCal(&HSLut, t_char_hs);
        t_he3fridge = LutCal(&ROXLut, t_he3fridge);
        t_he4pot = LutCal(&ROXLut, t_he4pot);

        if (t_lhe > T_LHE_MAX) {
            if (cycle_state != CRYO_CYCLE_OUT)
                blast_info("Auto Cycle: LHe is DRY!\n");
            heat_char = 0;
            heat_hs = 1;
            SET_VALUE(stateCycleAddr, CRYO_CYCLE_OUT);
        }
        else if (cycle_state == CRYO_CYCLE_COLD) {
            if ((t_he3fridge > CommandData.Cryo.cycle_start_temp && t_he4pot < CommandData.Cryo.cycle_pot_max) || *force_cycle) {
                *force_cycle = 0;
                SET_VALUE(stateCycleAddr, CRYO_CYCLE_HS_OFF);
                SET_VALUE(startCycleAddr, mcp_systime(NULL));
                heat_char = heat_hs = 0;
                blast_info("Auto Cycle: Turning charcoal heatswitch off.");
            }
            else {
                heat_char = 0;
                heat_hs = 1;
            }
        }
        else if (cycle_state == CRYO_CYCLE_HS_OFF) {
            if (((mcp_systime(NULL) - start_time) > CRYO_CYCLE_HS_TIMEOUT) || t_char_hs < T_CHAR_HS_COLD) {
                SET_VALUE(stateCycleAddr, CRYO_CYCLE_ON);
                heat_char = 1;
                heat_hs = 0;
                blast_info("Auto Cycle: Turning charcoal heat on.");
            }
            else {
                heat_char = heat_hs = 0;
            }
        }
        else if (cycle_state == CRYO_CYCLE_ON) {
            if (((mcp_systime(NULL) - start_time) > CommandData.Cryo.cycle_charcoal_timeout * 60)
                    || t_charcoal > CommandData.Cryo.cycle_charcoal_max) {
                SET_VALUE(startSetCycleAddr, mcp_systime(NULL));
                SET_VALUE(stateCycleAddr, CRYO_CYCLE_SETTLE);
                heat_char = 0;
                heat_hs = 0;
                blast_info("Auto Cycle: Charcoal heat off");
            }
            else if (t_he4pot > CommandData.Cryo.cycle_pot_max) {
                SET_VALUE(stateCycleAddr, CRYO_CYCLE_COOL);
                heat_char = 0;
                heat_hs = 1;
                blast_info("Auto Cycle: Pot out. Charcoal heat off, heatswitch on.");
            }
            else {
                heat_char = 1;
                heat_hs = 0;
            }
        }
        else if (cycle_state == CRYO_CYCLE_SETTLE) {
            if (((mcp_systime(NULL) - start_time) > overall_settle_timeout * 60)
                    || ((mcp_systime(NULL) - settle_start_time) > CommandData.Cryo.cycle_settle_timeout * 60)
                    || t_charcoal < CommandData.Cryo.cycle_charcoal_settle) {
                SET_VALUE(stateCycleAddr, CRYO_CYCLE_COOL);
                heat_char = 0;
                heat_hs = 1;
                if ((mcp_systime(NULL) - start_time) > overall_settle_timeout * 60)
                    blast_warn("Auto Cycle: aborting settle on overall timeout");
                blast_info("Auto Cycle: Charcoal heatswitch on");
            }
            else {
                heat_char = heat_hs = 0;
            }
        }
        else if (cycle_state == CRYO_CYCLE_COOL) {
            if ((t_he3fridge < t_he3fridge_cold) || ((mcp_systime(NULL) - start_time) > CRYO_CYCLE_COOL_TIMEOUT)) {
                SET_VALUE(stateCycleAddr, CRYO_CYCLE_COLD);
                heat_char = 0;
                heat_hs = 1;
                *force_cycle = 0; // clear any pending cycle commands...
                blast_info("Auto Cycle: Fridge is now cold!.");
            }
            else {
                heat_char = 0;
                heat_hs = 1;
            }
        }
        else if (cycle_state == CRYO_CYCLE_OUT) {
            if (t_lhe < T_LHE_MAX) {
                SET_VALUE(stateCycleAddr, CRYO_CYCLE_COLD);
                heat_char = 0;
                heat_hs = 1;
                blast_info("Auto Cycle: Activated.");
            }
        }
        else {
            blast_err("Auto Cycle: cycle_state: %i unknown!", cycle_state);
            SET_VALUE(stateCycleAddr, CRYO_CYCLE_COLD);
            heat_char = 0;
            heat_hs = 1;
        }
    }

    if (heat_char)
        *heatctrl |= HEAT_CHARCOAL;
    else
        *heatctrl &= ~HEAT_CHARCOAL;

    if (heat_hs)
        *heatctrl |= HEAT_CHARCOAL_HS;
    else
        *heatctrl &= ~HEAT_CHARCOAL_HS;

    return;
}

/*************************************************************************/
/* CryoControl: Control valves, heaters, and calibrator (a fast control) */
/*************************************************************************/
void CryoControl(int index)
{
    static channel_t* cryostateAddr;
    static channel_t* jfetSetOnAddr;
    static channel_t* jfetSetOffAddr;
    static channel_t* dig43DasAddr;
    static channel_t* potHwprAddr;
    static channel_t* potRawHwprAddr;
    static channel_t* potRefHwprAddr;

    double pot_hwpr_raw, pot_hwpr_ref, pot_hwpr;

    static int cryostate = 0;
    int heatctrl = 0, valvectrl = 0;

    /************** Set indices first time around *************/
    static int firsttime = 1;
    if (firsttime) {
        firsttime = 0;
        cryostateAddr = channels_find_by_name("cryostate");
        jfetSetOnAddr = channels_find_by_name("jfet_set_on");
        jfetSetOffAddr = channels_find_by_name("jfet_set_off");
        dig43DasAddr = channels_find_by_name("dig43_das");
        potHwprAddr = channels_find_by_name("pot_hwpr");
        potRawHwprAddr = channels_find_by_name("pot_raw_hwpr");
        potRefHwprAddr = channels_find_by_name("pot_ref_hwpr");
    }

    /********** Set Output Bits **********/
    if (CommandData.Cryo.heliumLevel) {
        heatctrl |= HEAT_HELIUM_LEVEL;
        if (CommandData.Cryo.heliumLevel > 0)
            CommandData.Cryo.heliumLevel--;
    }

    if (CommandData.Cryo.hwprPos) {
        heatctrl |= HEAT_HWPR_POS;
        if (CommandData.Cryo.hwprPos > 0)
            CommandData.Cryo.hwprPos--;
    }

    if (CommandData.Cryo.autoJFETheat) {
        if (JFETthermostat())
            heatctrl |= HEAT_JFET;
        cryostate |= CS_AUTO_JFET;
    }
    else {
        if (CommandData.Cryo.JFETHeat)
            heatctrl |= HEAT_JFET;
        cryostate &= 0xFFFF - CS_AUTO_JFET;
    }

    if (CommandData.Cryo.hsPot)
        heatctrl |= HEAT_POT_HS;

    if (CommandData.Cryo.BDAHeat)
        heatctrl |= HEAT_BDA;

    if (CommandData.Cryo.fridgeCycle)
        FridgeCycle(&heatctrl, &cryostate, 0, &CommandData.Cryo.force_cycle);
    else {
        FridgeCycle(&heatctrl, &cryostate, 1, NULL);
        if (CommandData.Cryo.charcoalHeater)
            heatctrl |= HEAT_CHARCOAL;
        if (CommandData.Cryo.hsCharcoal)
            heatctrl |= HEAT_CHARCOAL_HS;
    }

    if (CalLamp(index))
        heatctrl |= HEAT_CALIBRATOR;

    /* Control motorised valves -- latching relays */
    valvectrl = VALVE_POT_OPEN | VALVE_POT_CLOSE |
    VALVE_L_OPEN | VALVE_L_CLOSE;

    if (CommandData.Cryo.potvalve_open > 0) {
        valvectrl &= ~VALVE_POT_OPEN;
        cryostate |= CS_POTVALVE_OPEN;
        CommandData.Cryo.potvalve_open--;
    }
    else if (CommandData.Cryo.potvalve_close > 0) {
        valvectrl &= ~VALVE_POT_CLOSE;
        cryostate &= 0xFFFF - CS_POTVALVE_OPEN;
        CommandData.Cryo.potvalve_close--;
    }
    if (CommandData.Cryo.potvalve_on) {
        valvectrl |= VALVE_POT_ON;
        cryostate |= CS_POTVALVE_ON;
    }
    else {
        cryostate &= 0xFFFF - CS_POTVALVE_ON;
    }

    if (CommandData.Cryo.lvalve_open > 0) {
        valvectrl &= ~VALVE_L_OPEN;
        cryostate |= CS_LVALVE_OPEN;
        CommandData.Cryo.lvalve_open--;
    }
    else if (CommandData.Cryo.lvalve_close > 0) {
        valvectrl &= ~VALVE_L_CLOSE;
        cryostate &= 0xFFFF - CS_LVALVE_OPEN;
        CommandData.Cryo.lvalve_close--;
    }
    if (CommandData.Cryo.lhevalve_on) {
        valvectrl |= VALVE_LHe_ON;
        cryostate |= CS_LHeVALVE_ON;
    }
    else {
        cryostate &= 0xFFFF - CS_LHeVALVE_ON;
    }
    if (CommandData.Cryo.lnvalve_on) {
        valvectrl |= VALVE_LN_ON;
        cryostate |= CS_LNVALVE_ON;
    }
    else {
        cryostate &= 0xFFFF - CS_LNVALVE_ON;
    }

    //HWPR potentiometer logic
    pot_hwpr_raw = (double) GET_UINT32(potRawHwprAddr);
    pot_hwpr_raw -= 2147483648.0;	  //convert from 32-bit offset signed
    pot_hwpr_ref = (double) GET_UINT32(potRefHwprAddr);
    pot_hwpr_ref -= 2147483648.0;	  //convert from 32-bit offset signed
    pot_hwpr = pot_hwpr_raw / pot_hwpr_ref;
    //don't update hwpr_pot when not pulsing the read, or when reading too high
    if ((heatctrl & HEAT_HWPR_POS) && (pot_hwpr < HWPR_POT_MAX)) {
        SET_VALUE(potHwprAddr, (int )(pot_hwpr * 65535.0));
    }

    if (index == 0) {
        SET_VALUE(cryostateAddr, cryostate);
        SET_VALUE(jfetSetOnAddr, CommandData.Cryo.JFETSetOn * 100);
        SET_VALUE(jfetSetOffAddr, CommandData.Cryo.JFETSetOff * 100);
    }
    SET_VALUE(dig43DasAddr, (heatctrl << 8) | valvectrl);
}

/************************************************************************/
/*                                                                      */
/*   BiasControl: Digital IO with the Bias Generator Card               */
/*                                                                      */
/************************************************************************/
void BiasControl()
{
    static channel_t* amplBiasAddr[5];
    static channel_t* rampEnaBiasAddr;
    static channel_t* rampAmplBiasAddr;
    static channel_t* stepEnaBiasAddr;
    static channel_t* stepStartBiasAddr;
    static channel_t* stepEndBiasAddr;
    static channel_t* stepNBiasAddr;
    static channel_t* stepTimeBiasAddr;
    static channel_t* stepPulLenBiasAddr;
    static channel_t* stepArrayBiasAddr;
    static int i_arr = 3;
    static int k = 0;
    static int step_size = 1;
    static int dk = 1;
    static int end = 0;
    static int start = 0;
    static int kp = 0;
    static int pulse_len_k = 0;
    int bias = 0;

    int i;
    int isBiasRamp;

    /******** Obtain correct indexes the first time here ***********/
    static int firsttime = 1;
    if (firsttime) {
        firsttime = 0;
        amplBiasAddr[0] = channels_find_by_name("ampl_500_bias");
        amplBiasAddr[1] = channels_find_by_name("ampl_350_bias");
        amplBiasAddr[2] = channels_find_by_name("ampl_250_bias");
        amplBiasAddr[3] = channels_find_by_name("ampl_rox_bias");
        amplBiasAddr[4] = channels_find_by_name("ampl_x_bias");
        rampEnaBiasAddr = channels_find_by_name("ramp_ena_bias");
        rampAmplBiasAddr = channels_find_by_name("ramp_ampl_bias");
        stepEnaBiasAddr = channels_find_by_name("step_ena_bias");
        stepStartBiasAddr = channels_find_by_name("step_start_bias");
        stepEndBiasAddr = channels_find_by_name("step_end_bias");
        stepNBiasAddr = channels_find_by_name("step_n_bias");
        stepTimeBiasAddr = channels_find_by_name("step_time_bias");
        stepPulLenBiasAddr = channels_find_by_name("step_pul_len_bias");
        stepArrayBiasAddr = channels_find_by_name("step_array_bias");
    }

    /* Check to make sure that the user selected an array.  0 means step all arrays.*/
    if (CommandData.Bias.biasStep.do_step) {
        switch (CommandData.Bias.biasStep.arr_ind) {
            case 250:
                i_arr = 2;
                break;
            case 350:
                i_arr = 1;
                break;
            case 500:
                i_arr = 0;
                break;
            case 0:
                i_arr = 3; // i.e. all wavelengths
                break;
            default:
                CommandData.Bias.biasStep.do_step = 0; // don't step
                break;
        }
    }

    /************* Set the Bias Levels *******/
    for (i = 0; i < 5; i++) {
        if (CommandData.Bias.setLevel[i]) {
            SET_VALUE(amplBiasAddr[i], CommandData.Bias.bias[i] << 1);
            CommandData.Bias.setLevel[i] = 0;
            CommandData.Bias.biasStep.do_step = 0;
        }
    }
    if (CommandData.Bias.biasStep.do_step) {
        if (k == 0) {
            start = CommandData.Bias.biasStep.start;
            end = CommandData.Bias.biasStep.end;
            step_size = (end - start) / CommandData.Bias.biasStep.nsteps;

            if (step_size == 0) { // minimum step size is 1
                if (end >= start) {
                    step_size = 1;
                }
                if (end < start) {
                    step_size = -1;
                }
            }
            end += step_size;
            dk = (unsigned int) (CommandData.Bias.biasStep.dt * SR / 1000);
            if (dk == 0)
                dk = 1;
            pulse_len_k = (unsigned int) (CommandData.Bias.biasStep.pulse_len * SR / 1000);

            /* kp sets when in the step the cal pulse will be sent */
            /* NOTE: if pulse_len= 0, no pulse is sent. */
            kp = dk - 4 * pulse_len_k;

            if (kp < 0)
                kp = 1;
        }

        bias = start + (k / dk) * step_size;

        if (step_size > 0) {
            if (bias >= end)
                CommandData.Bias.biasStep.do_step = 0;
            if (bias > 32767) {
                bias = 32767;
                CommandData.Bias.biasStep.do_step = 0;
            }

        }
        else {
            if (bias <= end) {
                CommandData.Bias.biasStep.do_step = 0;
            }
            if (bias < 1) {
                bias = 1;
                CommandData.Bias.biasStep.do_step = 0;
            }
        }

        SET_VALUE(stepEnaBiasAddr, CommandData.Bias.biasStep.do_step);
        SET_VALUE(stepStartBiasAddr, CommandData.Bias.biasStep.start << 1);
        SET_VALUE(stepEndBiasAddr, CommandData.Bias.biasStep.end << 1);
        SET_VALUE(stepNBiasAddr, CommandData.Bias.biasStep.nsteps);
        SET_VALUE(stepTimeBiasAddr, CommandData.Bias.biasStep.dt);
        SET_VALUE(stepPulLenBiasAddr, CommandData.Bias.biasStep.pulse_len);
        SET_VALUE(stepArrayBiasAddr, CommandData.Bias.biasStep.arr_ind);
        if (i_arr >= 0 && i_arr < 3) {
            SET_VALUE(amplBiasAddr[i_arr], bias << 1);
        }
        else {
            for (i = 0; i <= 2; i++)
                SET_VALUE(amplBiasAddr[i], bias << 1);
        }

        /* Send a cal pulse at 4x the width of the cal pulse before the next step.*/
        /* pulse_len = 0 means don't send a cal pulse */
        if (k % dk == kp && pulse_len_k > 0) {
            CommandData.Cryo.calibrator = pulse;
            CommandData.Cryo.calib_pulse = pulse_len_k;
        }
        k++;
    }
    else {
        k = 0;
        /********** set Bias (ramp)  *******/
        isBiasRamp = GET_UINT16(rampAmplBiasAddr);
        if ((isBiasRamp && CommandData.Bias.biasRamp == 0) || (!isBiasRamp && CommandData.Bias.biasRamp == 1)) {
            SET_VALUE(rampEnaBiasAddr, CommandData.Bias.biasRamp);
        }

    }
}
