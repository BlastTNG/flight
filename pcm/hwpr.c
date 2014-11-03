/* hwpr: part of the Spider master control program
 *
 * This software is copyright (C) 2012 University of Toronto
 *
 * This file is part of pcm.
 *
 * pcm is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * pcm is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "mcp.h"

#include <unistd.h>
#include <stdlib.h>
#include <math.h>

#include "hwpr.h"
#include "phytron.h"
#include "command_struct.h"
#include "pointing_struct.h"
#include "tx.h"

const char *hwp_name[NHWP] = {"HWP S1", "HWP S2", "HWP S3",
  "HWP S4", "HWP S5", "HWP S6"};
const char *hwp_id[NHWP] = {"0X", "0Y", "1X", "1Y", "2X", "2Y"};
const int hwp_nteeth[NHWP] = {465, 464, 463, 464, 463, 464};


#define	MAX_SERIAL_ERRORS 20		// after this many, repoll bus
#define POLL_TIMEOUT 300        // in units of slow frames
//extend timeout for seancal tests with incomplete set of phytrons
//#define POLL_TIMEOUT 30000        // in units of slow frames
static int poll_timeout = 1;

static unsigned int actuators_init = 0;	// bitfield for when actuators usable

static struct phytron bus;

static struct {
  int pos;
  int shaft_count;
  int direction;
} hwp_data[NHWP];

//determine relative move to next HWP position in sequence
static double hwpToNext(int who)
{
  //for now, operate in a dumb blind way
  return 22.5;
}

static void DoHWP()
{
  int i;
  double to_next;

  for (i=0; i<NHWP; i++) {
    // update the HWP move parameters
    Phytron_SetVel(&bus, i, CommandData.hwp.vel);
    Phytron_SetIMove(&bus, i, CommandData.hwp.move_i);

    //execute controls for the given mode
    switch (CommandData.hwp.mode[i]) {
      case hwp_m_panic:
        Phytron_Stop_All(&bus);
        break;
      case hwp_m_halt:
        Phytron_Stop(&bus, i);
        break;
      case hwp_m_rel_move:
        hwp_data[i].direction = (CommandData.hwp.delta[i] >= 0);
        Phytron_Move(&bus, i, CommandData.hwp.delta[i]);
        break;
      case hwp_m_step:
        to_next = hwpToNext(i);
        hwp_data[i].direction = (to_next >= 0);
        Phytron_Move(&bus, i, to_next);
        break;
      case hwp_m_sleep:
      default:
        break;
    }

    //reset state
    CommandData.hwp.mode[i] = hwp_m_sleep;

    //read absolute step counters
    Phytron_ReadInt(&bus, i, "P21R", &hwp_data[i].pos);
  }
}

void StoreHWPBus(void)
{
  int i;
  static int firsttime = 1;
  static struct NiosStruct* velHwpAddr;
  static struct NiosStruct* iMoveHwpAddr;
  static struct NiosStruct* statusHwpAddr;
  static struct NiosStruct* posHwpAddr[NHWP];
  static struct NiosStruct* hwpBiasAddr;
  static struct NiosStruct* phaseHwpAddr;
  double degrees;

  if (firsttime)
  {
    firsttime = 0;
    velHwpAddr = GetNiosAddr("vel_hwp");
    iMoveHwpAddr = GetNiosAddr("i_move_hwp");
    statusHwpAddr = GetNiosAddr("status_hwp");
    hwpBiasAddr = GetNiosAddr("hwp_bias");
    for (i=0; i<NHWP; i++) {
      char namebuf[100];
      sprintf(namebuf, "pos_x%i_hwp", i+1);
      posHwpAddr[i] = GetNiosAddr(namebuf);
    }
    phaseHwpAddr = GetNiosAddr("phase_hwp");
  }
  WriteData(hwpBiasAddr, CommandData.hwp.bias_mask, NIOS_QUEUE);
  WriteCalData(velHwpAddr, CommandData.hwp.vel, NIOS_QUEUE);
  WriteCalData(iMoveHwpAddr, CommandData.hwp.move_i, NIOS_QUEUE);
  WriteData(statusHwpAddr, actuators_init, NIOS_QUEUE);
  for (i=0; i<NHWP; i++) {
    degrees = Phytron_S2D(&bus, i, hwp_data[i].pos);
    while (degrees < 0) degrees += 360.;
    while (degrees > 360.) degrees -= 360.;
    WriteCalData(posHwpAddr[i], degrees, NIOS_QUEUE);
  }

  WriteCalData(phaseHwpAddr, CommandData.hwp.phase, NIOS_QUEUE);

  if (poll_timeout > 0) poll_timeout--;
}

void countHWPEncoder(int index)
{
  int i, j;
  static int firsttime = 1;
  //TODO upgrade to handle reading all NHWP encoders
  static struct BiPhaseStruct* encSHwpAddr[NHWP];
  static struct NiosStruct* encCntHwpAddr[NHWP];

  double degrees;

  static double data[NHWP][5] = {{0}};

  //sanity check on the voltage change between min/max ticks
  //TODO setup system to actually measure noise level when not moving
  double noise_level = 1.e-5; //volts, exaggerated
  double min_amp = 20 * noise_level;
  static double last_tick_data[NHWP];

  //sanity check the number of samples between ticks: half an expected period
  double samples_per_tick = ( (360.0/465.0)/40.0 )
      * (ACSData.bbc_rate / CommandData.hwp.vel);
  int min_tick_spacing = samples_per_tick / 2.0;
  static int since_last_tick[NHWP];

  if (firsttime)
  {
    firsttime = 0;
    for (i=0; i<NHWP; i++) {
      char namebuf[100];
      sprintf(namebuf, "enc_s_x%i_hwp", i+1);
      encSHwpAddr[i] = GetBiPhaseAddr(namebuf);
      sprintf(namebuf, "enc_cnt_x%i_hwp", i+1);
      encCntHwpAddr[i] = GetNiosAddr(namebuf);

      last_tick_data[i] = -20.;  //initialize far from all real values
      since_last_tick[i] = min_tick_spacing;  //allow detection immediately
    }
  }

  for (i=0; i<NHWP; i++) {
    //shift new data into buffer
    for (j=4; j>0; j--) data[i][j] = data[i][j-1];
    data[i][0] = ReadCalData(encSHwpAddr[i]);
    since_last_tick[i]++;

    //check to see if this is a minimum or maximium
    //if ( ((data[i][0] < data[i][2]) && (data[i][4] < data[i][2]))
        //|| ((data[i][0] > data[i][2]) && (data[i][4] > data[i][2])) )
    if ( ((data[i][1] < data[i][2]) && (data[i][3] < data[i][2]))
        || ((data[i][1] > data[i][2]) && (data[i][3] > data[i][2])) )
    {
      if (fabs(data[i][2] - last_tick_data[i]) > min_amp) {
        if (since_last_tick[i] >= min_tick_spacing) {
          //tick.
          since_last_tick[i] = 0;
          last_tick_data[i] = data[i][2];
          if (hwp_data[i].direction) hwp_data[i].shaft_count++;
          else hwp_data[i].shaft_count--;
        }
      }
    }

    //this function is called at fast rate. Only write data at slow rate
    if (index == 0) {
      //write the count (converted to degrees) to the frame
      //(200 steps/rev) / (40 counts/rev) = (5 steps/count)
      degrees = Phytron_S2D(&bus, 0,
          hwp_data[i].shaft_count * 5 * bus.stepper[i].usteps);
      while (degrees < 0) degrees += 360.;
      while (degrees > 360.) degrees -= 360.;
      WriteCalData(encCntHwpAddr[i], degrees, NIOS_QUEUE);
    }
  }
}

//create a thread for this function
void StartHWP(void)
{
  int all_ok = 0;
  int i;
  int j=0;
  int my_cindex = 0;
  int is_init = 0;
  int first_time=1;


  nameThread("HWP");

  //Initialize global data
  for (i=0; i<NHWP; i++) {
    hwp_data[i].pos = 0;
    hwp_data[i].shaft_count = 0;
    hwp_data[i].direction = 1;
  }

  //Wait until in charge before doing any serial port stuff
  while (!InCharge) {
    if (first_time) {
      first_time = 0;
    }
    usleep(1000000);
    //if dead reckoning is implemented, this will be needed again
    //if (BLASTBusUseful) {
      //SyncDR();	     // get encoder absolute state from the ICC
    //}
    CommandData.hwp.caddr[my_cindex] = -1; // prevent multi-execution by NICC
  }

  //Initialize the bus
  first_time = 1;
  is_init = 0;
  while (!is_init) {
    if (Phytron_Init(&bus, HWP_BUS, "", HWP_CHATTER) == PH_ERR_OK) {
      is_init = 1;
    }
    usleep(100000);
    if (j == 10) {
      bprintf(info,"Bus failed to initialize after 10 attempts");
    }
    j++;
  }
  
  if (j>11) {
    bprintf(info,"Bus initialized on %ith attempt",j-1);
  }

  //Setup stepper parameters
  for (i=0; i<NHWP; i++) {
    Phytron_Add(&bus, i, hwp_id[i], hwp_name[i]);
    Phytron_SetGear(&bus, i, hwp_nteeth[i]);
  }

  all_ok = !(Phytron_Poll(&bus) & PH_ERR_POLL);

  for (;;) {
    // Repoll bus if necessary
    if (CommandData.hwp.force_repoll || bus.err_count > MAX_SERIAL_ERRORS) {
      for (i=0; i<NHWP; i++) Phytron_ForceRepoll(&bus, i);
      //supress non-error messages, except during manual repoll
      if (!CommandData.hwp.force_repoll) bus.chatter = PH_CHAT_ERR;
      all_ok = !(Phytron_Poll(&bus) & PH_ERR_POLL);
      bus.chatter = HWP_CHATTER;
      CommandData.hwp.force_repoll = 0;
    }

    if (poll_timeout <= 0 && !all_ok /*&& bus is on*/) {//TODO fix "bus is on"
      //suppress non-error messages during repoll
      bus.chatter = PH_CHAT_ERR;
      all_ok = !(Phytron_Poll(&bus) & PH_ERR_POLL);
      bus.chatter = HWP_CHATTER;
      poll_timeout = POLL_TIMEOUT;
    }    

    // Send the uplinked command, if any
    my_cindex = GETREADINDEX(CommandData.hwp.cindex);
    if (CommandData.hwp.caddr[my_cindex] >= 0) {
      //bprintf(info, "Sending command \"%s\" to \"%s\"\n",
      //    CommandData.hwp.command[my_cindex], 
      //    bus.stepper[CommandData.hwp.caddr[my_cindex]].name);
      //increase print level for uplinked manual commands
      //bus.chatter = PH_CHAT_BUS;
      if (CommandData.hwp.command[my_cindex][0] == '\\') {
        //'\\' indicates NAComm. Skip the '\\' character itself
        Phytron_NAComm(&bus, CommandData.hwp.caddr[my_cindex],
            CommandData.hwp.command[my_cindex] + 1);
      } else {
        Phytron_Comm(&bus, CommandData.hwp.caddr[my_cindex],
            CommandData.hwp.command[my_cindex]);
      }
      CommandData.hwp.caddr[my_cindex] = -1;
      //bus.chatter = HWP_CHATTER;
    }

    for (i=0; i<NHWP; i++) {
      if (Phytron_IsUsable(&bus, i)) {
        actuators_init |= 0x1 << i;
      } else {
        Phytron_ForceRepoll(&bus, i);
        all_ok = 0;
        actuators_init &= ~(0x1 << i);
      }
    }

    if (actuators_init) DoHWP();  //if any steppers are working, do hwp

    usleep(10000);

  }
}

