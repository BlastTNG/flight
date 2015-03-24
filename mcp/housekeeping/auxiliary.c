/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
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

#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/statvfs.h>

#include "mcp.h"
#include <mputs.h>

#include <therm_heater.h>
#include "channels_tng.h"
#include "tx.h"
#include "command_struct.h"
#include "pointing_struct.h"
#include "chrgctrl.h"
#include "lut.h"

extern short int InCharge; /* tx.c */

/* ACS2 digital signals */
#define BAL_DIR      0x01  /* ACS2 Group 2 Bit 1 */
#define BAL_VALV     0x02  /* ACS2 Group 2 Bit 2 */
#define BAL_HEAT     0x04  /* ACS2 Group 2 Bit 3 - DAC */

#define SBSC_HEAT    0x01  /* ACS1_D Spare-0 */

#define PUMP_MAX 26214      /*  3.97*2.0V   */
#define PUMP_MIN  3277      /*  3.97*0.25V   */

#define PUMP_ZERO 32773


/************************************************************************/
/*    ControlPumpHeat:  Controls balance system pump temp               */
/************************************************************************/

static int ControlPumpHeat(int bits_bal)
{

  static channel_t *tBoxBalAddr, *vtPumpBalAddr;
  static struct LutType tPumpBalLut =
     {"/data/etc/blast/thermistor.lut", 0, NULL, NULL, 0};
  static int firsttime = 1;

  double temp1, temp2;

  if (firsttime) {
    firsttime = 0;
    tBoxBalAddr = channels_find_by_name("t_box_bal");
    vtPumpBalAddr = channels_find_by_name("vt_pump_bal");  
    LutInit(&tPumpBalLut);
  }


  temp1 = (double)GET_UINT16(tBoxBalAddr);
  temp2 = (double)GET_UINT16(vtPumpBalAddr);
  
  temp1 = calibrate_ad590(temp1) - 273.15;
  temp2 = calibrate_thermister(temp2) - 273.15;
 
  if (CommandData.pumps.heat_on) {
    if (temp1 < CommandData.pumps.heat_tset) {
      bits_bal |= BAL_HEAT;  /* set heat bit */
    } else {
      bits_bal &= (0xFF - BAL_HEAT); /* clear heat bit */
    }
  } else {
      bits_bal &= (0xFF - BAL_HEAT); /* clear heat bit */
  }

  return bits_bal;

}




/*********************/
/* ISC Pulsing stuff */
void CameraTrigger(int which)
{
  static int firsttime = 1;
  static channel_t* TriggerAddr[2];
  static int delay[2] = {0, 0};
  static int waiting[2] = {0, 0};
#if SYNCHRONOUS_CAMERAS
  static int cameras_ready = 0;
#endif
  char swhich[4] = "ISC";
  if (which) swhich[0] = 'O';

  if (firsttime) {
    firsttime = 0;
    //NB before renaming, 0 was osc, 1 was isc. I think that was wrong
    TriggerAddr[0] = channels_find_by_name("trigger_isc");
    TriggerAddr[1] = channels_find_by_name("trigger_osc");
  }

  //TODO: Add Camera Triggering
}

/*****************************************************************/
/*                                                               */
/*   Control the pumps and the lock                              */
/*                                                               */
/*****************************************************************/
void ControlAuxMotors()
{
  static channel_t* levelOnBalAddr, *levelOffBalAddr;
  static channel_t* levelTargetBalAddr;
  static channel_t* gainBalAddr;
  static channel_t* bitsBalAddr;

  int bits_bal = 0;

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    bitsBalAddr = channels_find_by_name("bits_bal");
    levelOnBalAddr = channels_find_by_name("level_on_bal");
    levelOffBalAddr = channels_find_by_name("level_off_bal");
    levelTargetBalAddr = channels_find_by_name("level_target_bal");
    gainBalAddr = channels_find_by_name("gain_bal");
  }

  /* Run Heating card, maybe */
  bits_bal = ControlPumpHeat(bits_bal);
  
  SET_VALUE(levelOnBalAddr, CommandData.pumps.level_on_bal);
  SET_VALUE(levelOffBalAddr, CommandData.pumps.level_off_bal);
  SET_VALUE(levelTargetBalAddr, (CommandData.pumps.level_target_bal + 1990.13*5.));
  SET_VALUE(gainBalAddr, (int)(CommandData.pumps.gain_bal * 1000.));
  SET_VALUE(bitsBalAddr, bits_bal);

}

/* create latching relay pulses, and update enable/disbale levels */
/* actbus/steppers enable is handled separately in StoreActBus() */
void ControlPower(void) {

  static int firsttime = 1;
  static channel_t* latchingAddr[2];
  static channel_t* switchGyAddr;
  static channel_t* switchMiscAddr;
  int latch0 = 0, latch1 = 0, gybox = 0, misc = 0;
  int i;

  if (firsttime) {
    firsttime = 0;
    latchingAddr[0] = channels_find_by_name("latch0");
    latchingAddr[1] = channels_find_by_name("latch1");
    switchGyAddr = channels_find_by_name("switch_gy");
    switchMiscAddr = channels_find_by_name("switch_misc");
  }

  if (CommandData.power.hub232_off) {
    if (CommandData.power.hub232_off > 0) CommandData.power.hub232_off--;
    misc |= 0x08;
  }

  if (CommandData.power.charge.set_count > 0) {
    CommandData.power.charge.set_count--;
    if (CommandData.power.charge.set_count < LATCH_PULSE_LEN) misc |= 0x0040;
  }
  if (CommandData.power.charge.rst_count > 0) {
    CommandData.power.charge.rst_count--;
    if (CommandData.power.charge.rst_count < LATCH_PULSE_LEN) misc |= 0x0080;
  }

  for (i=0; i<6; i++) {
    if (CommandData.power.gyro_off[i] || CommandData.power.gyro_off_auto[i]) {
      if (CommandData.power.gyro_off[i] > 0) 
	CommandData.power.gyro_off[i]--;
      if (CommandData.power.gyro_off_auto[i] > 0) 
	CommandData.power.gyro_off_auto[i]--;
      gybox |= 0x01 << i;
    }
  }

  if (CommandData.power.gybox_off) {
    if (CommandData.power.gybox_off > 0) CommandData.power.gybox_off--;
    gybox |= 0x80;
  }

  if (CommandData.power.sc_tx.set_count > 0) {
    CommandData.power.sc_tx.set_count--;
    if (CommandData.power.sc_tx.set_count < LATCH_PULSE_LEN) latch0 |= 0x0001;
  }
  if (CommandData.power.sc_tx.rst_count > 0) {
    CommandData.power.sc_tx.rst_count--;
    if (CommandData.power.sc_tx.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0002;
  }
  if (CommandData.power.das.set_count > 0) {
    CommandData.power.das.set_count--;
    if (CommandData.power.das.set_count < LATCH_PULSE_LEN) latch0 |= 0x0004;
  }
  if (CommandData.power.das.rst_count > 0) {
    CommandData.power.das.rst_count--;
    if (CommandData.power.das.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0008;
  }
  if (CommandData.power.isc.set_count > 0) {
    CommandData.power.isc.set_count--;
    if (CommandData.power.isc.set_count < LATCH_PULSE_LEN) latch0 |= 0x0010;
  }
  if (CommandData.power.isc.rst_count > 0) {
    CommandData.power.isc.rst_count--;
    if (CommandData.power.isc.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0020;
  }
  if (CommandData.power.osc.set_count > 0) {
    CommandData.power.osc.set_count--;
    if (CommandData.power.osc.set_count < LATCH_PULSE_LEN) latch0 |= 0x0040;
  }
  if (CommandData.power.osc.rst_count > 0) {
    CommandData.power.osc.rst_count--;
    if (CommandData.power.osc.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0080;
  }
  if (CommandData.power.sbsc.set_count > 0) {
    CommandData.power.sbsc.set_count--;
    if (CommandData.power.sbsc.set_count < LATCH_PULSE_LEN) latch0 |= 0x0100;
  }
  if (CommandData.power.sbsc.rst_count > 0) {
    CommandData.power.sbsc.rst_count--;
    if (CommandData.power.sbsc.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0200;
  }
  if (CommandData.power.rw.set_count > 0) {
    CommandData.power.rw.set_count--;
    if (CommandData.power.rw.set_count < LATCH_PULSE_LEN) latch0 |= 0x0400;
  }
  if (CommandData.power.rw.rst_count > 0) {
    CommandData.power.rw.rst_count--;
    if (CommandData.power.rw.rst_count < LATCH_PULSE_LEN) latch0 |= 0x0800;
  }
  if (CommandData.power.piv.set_count > 0) {
    CommandData.power.piv.set_count--;
    if (CommandData.power.piv.set_count < LATCH_PULSE_LEN) latch0 |= 0x1000;
  }
  if (CommandData.power.piv.rst_count > 0) {
    CommandData.power.piv.rst_count--;
    if (CommandData.power.piv.rst_count < LATCH_PULSE_LEN) latch0 |= 0x2000;
  }
  if (CommandData.power.elmot.set_count > 0) {
    CommandData.power.elmot.set_count--;
    if (CommandData.power.elmot.set_count < LATCH_PULSE_LEN) latch0 |= 0x4000;
  }
  if (CommandData.power.elmot.rst_count > 0) {
    CommandData.power.elmot.rst_count--;
    if (CommandData.power.elmot.rst_count < LATCH_PULSE_LEN) latch0 |= 0x8000;
  }
  if (CommandData.power.bi0.set_count > 0) {
    CommandData.power.bi0.set_count--;
    if (CommandData.power.bi0.set_count < LATCH_PULSE_LEN) latch1 |= 0x0001;
  }
  if (CommandData.power.bi0.rst_count > 0) {
    CommandData.power.bi0.rst_count--;
    if (CommandData.power.bi0.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0002;
  }
  if (CommandData.power.rx_main.set_count > 0) {
    CommandData.power.rx_main.set_count--;
    if (CommandData.power.rx_main.set_count < LATCH_PULSE_LEN) latch1 |= 0x0004;
  }
  if (CommandData.power.rx_main.rst_count > 0) {
    CommandData.power.rx_main.rst_count--;
    if (CommandData.power.rx_main.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0008;
  }
  if (CommandData.power.rx_hk.set_count > 0) {
    CommandData.power.rx_hk.set_count--;
    if (CommandData.power.rx_hk.set_count < LATCH_PULSE_LEN) latch1 |= 0x5050;
  }
  if (CommandData.power.rx_hk.rst_count > 0) {
    CommandData.power.rx_hk.rst_count--;
    if (CommandData.power.rx_hk.rst_count < LATCH_PULSE_LEN) latch1 |= 0xa0a0;
  }
  if (CommandData.power.rx_amps.set_count > 0) {
    CommandData.power.rx_amps.set_count--;
    if (CommandData.power.rx_amps.set_count < LATCH_PULSE_LEN) latch1 |= 0x0500;
  }
  if (CommandData.power.rx_amps.rst_count > 0) {
    CommandData.power.rx_amps.rst_count--;
    if (CommandData.power.rx_amps.rst_count < LATCH_PULSE_LEN) latch1 |= 0x0a00;
  }

  SET_VALUE(latchingAddr[0], latch0);
  SET_VALUE(latchingAddr[1], latch1);
  SET_VALUE(switchGyAddr, gybox);
  SET_VALUE(switchMiscAddr, misc);
}

void VideoTx(void)
{
  static channel_t* bitsVtxAddr;  
  static int firsttime =1;
  int vtx_bits = 0;

  if (firsttime) {
    firsttime = 0;
    bitsVtxAddr = channels_find_by_name("bits_vtx");
  }

  if (CommandData.vtx_sel[0] == vtx_sbsc) vtx_bits |= 0x3;
  else if (CommandData.vtx_sel[0] == vtx_osc) vtx_bits |= 0x1;
  if (CommandData.vtx_sel[1] == vtx_sbsc) vtx_bits |= 0xc;
  else if (CommandData.vtx_sel[1] == vtx_isc) vtx_bits |= 0x4;

  SET_VALUE(bitsVtxAddr, vtx_bits);
}

static void SensorReader(void)
{
  int data;
  int nr;
  struct statvfs vfsbuf;
  int sensor_error = 0;

  FILE *stream;

  nameThread("Sensor");
  bputs(startup, "Startup\n");

  while (1) {
    if ((stream = fopen("/sys/bus/i2c/devices/0-002d/temp1_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp1 = data / 10;
      fclose(stream);
    } else {
      if (!sensor_error)
        berror(warning, "Cannot read temp1 from I2C bus");
      sensor_error = 5;
    }

    if ((stream = fopen("/sys/bus/i2c/devices/0-002d/temp2_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp2 = data / 10;
      fclose(stream);
    } else {
      if (!sensor_error)
        berror(warning, "Cannot read temp2 from I2C bus");
      sensor_error = 5;
    }

    if ((stream = fopen("/sys/bus/i2c/devices/0-002d/temp3_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp3 = data / 10;
      fclose(stream);
    } else {
      if (!sensor_error)
        berror(warning, "Cannot read temp3 from I2C bus");
      sensor_error = 5;
    }

    if (statvfs("/data", &vfsbuf))
      berror(warning, "Cannot stat filesystem");
    else {
      /* vfsbuf.f_bavail is the # of blocks, the blocksize is vfsbuf.f_bsize
       * which, in this case is 4096 bytes, so CommandData.df ends up in units
       * of 4000kb */
      CommandData.df = vfsbuf.f_bavail / 1000;
    }

    if (sensor_error)
      sensor_error--;

    usleep(100000);
  }
}
