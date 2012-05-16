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
#include "tx.h"

const char *hwp_name[NHWP] = {"HWP S1", "HWP S2", "HWP S3",
  "HWP S4", "HWP S5", "HWP S6"};
const char *hwp_id[NHWP] = {"0X", "0Y", "1X", "1Y", "2X", "2Y"};
const int hwp_nteeth[NHWP] = {465, 464, 463, 464, 463, 464};


#define	MAX_SERIAL_ERRORS 20		// after this many, repoll bus
#define POLL_TIMEOUT 25         // in units of slow frames
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

  // update the HWP move parameters
  for (i=0; i<NHWP; i++) {
    Phytron_SetVel(&bus, i, CommandData.hwp.vel);
    Phytron_SetIMove(&bus, i, CommandData.hwp.move_i);
  }

  //execute controls for the given mode
  switch (CommandData.hwp.mode) {
    case hwp_m_panic:
      Phytron_Stop_All(&bus);
      break;
    case hwp_m_halt:
      if (CommandData.hwp.who >= NHWP || CommandData.hwp.who < 0)
        for (i=0; i<NHWP; i++) Phytron_Stop(&bus, i);
      else Phytron_Stop(&bus, CommandData.hwp.who);
      break;
    case hwp_m_rel_move:
      if (CommandData.hwp.who >= NHWP || CommandData.hwp.who < 0) {
        for (i=0; i<NHWP; i++) {
          hwp_data[i].direction = (CommandData.hwp.delta >= 0);
          Phytron_Move(&bus, i, CommandData.hwp.delta);
        }
      } else {
        hwp_data[CommandData.hwp.who].direction = (CommandData.hwp.delta >= 0);
        Phytron_Move(&bus, CommandData.hwp.who, CommandData.hwp.delta);
      }
      break;
    case hwp_m_step:
      for (i=0; i<NHWP; i++) {
        to_next = hwpToNext(i);
        hwp_data[i].direction = (to_next >= 0);
        Phytron_Move(&bus, i, to_next);
      }
      break;
    case hwp_m_sleep:
    default:
      break;
  }

  //reset state
  CommandData.hwp.mode = hwp_m_sleep;

  //read absolute step counters
  for (i=0; i<NHWP; i++) {
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
  //TODO change DSP code so that hwp phase requires only one channel
  static struct NiosStruct* phaseHwpAddr[25];
  double degrees;

  if (firsttime)
  {
    firsttime = 0;
    velHwpAddr = GetNiosAddr("vel_hwp");
    iMoveHwpAddr = GetNiosAddr("i_move_hwp");
    statusHwpAddr = GetNiosAddr("status_hwp");
    for (i=0; i<NHWP; i++) {
      char namebuf[100];
      sprintf(namebuf, "pos_%i_hwp", i+1);
      posHwpAddr[i] = GetNiosAddr(namebuf);
    }
    for (i=0; i<25; i++) {
      char namebuf[100];
      sprintf(namebuf, "phase_%02i_hwp", i);
      phaseHwpAddr[i] = GetNiosAddr(namebuf);
    }
  }

  WriteCalData(velHwpAddr, CommandData.hwp.vel, NIOS_QUEUE);
  WriteCalData(iMoveHwpAddr, CommandData.hwp.move_i, NIOS_QUEUE);
  WriteData(statusHwpAddr, actuators_init, NIOS_QUEUE);
  for (i=0; i<NHWP; i++) {
    degrees = Phytron_S2D(&bus, i, hwp_data[i].pos);
    while (degrees < 0) degrees += 360.;
    while (degrees > 360.) degrees -= 360.;
    WriteCalData(posHwpAddr[i], degrees, NIOS_QUEUE);
  }

  for (i=0; i<25; i++)
    WriteCalData(phaseHwpAddr[i], CommandData.hwp.phase, NIOS_QUEUE);

  if (poll_timeout > 0) poll_timeout--;
}

void countHWPEncoder()
{
  int i;
  static int firsttime = 1;
  //TODO upgrade to handle reading all NHWP encoders
  static struct BiPhaseStruct* encSHwpAddr[NHWP];
  static struct NiosStruct* encCntHwpAddr[NHWP];

  double degrees;

  static double data_shaft[5] = {0};

  //sanity check on the voltage change between min/max ticks
  //TODO setup system to actually measure noise level when not moving
  double noise_level = 1.e-5; //volts, exaggerated
  double min_amp = 20 * noise_level;
  static double last_tick_data = -20.;  //initialize far from all real values

  //sanity check the number of samples between ticks: half an expected period
  int samples_per_tick = ceil(( (360.0/465.0)/40.0 )
      * (SR / CommandData.hwp.vel));
  int min_tick_spacing = ceil(samples_per_tick / 2.0);
  static int since_last_tick = 0;

  if (firsttime)
  {
    firsttime = 0;
    for (i=0; i<NHWP; i++) {
      char namebuf[100];
      sprintf(namebuf, "enc_s_%i_hwp", i+1);
      encSHwpAddr[i] = GetBiPhaseAddr(namebuf);
      sprintf(namebuf, "enc_cnt_%i_hwp", i+1);
      encCntHwpAddr[i] = GetNiosAddr(namebuf);
    }
  }

  //shift new data into buffer
  for (i=4; i>0; i--) data_shaft[i] = data_shaft[i-1];
  data_shaft[0] = ReadCalData(encSHwpAddr[0]);
  since_last_tick++;

  //check to see if this is a minimum or maximium
  if ( ((data_shaft[0] < data_shaft[2]) && (data_shaft[4] < data_shaft[2]))
      || ((data_shaft[0] > data_shaft[2]) && (data_shaft[4] > data_shaft[2])) )
  {
    if (fabs(data_shaft[1] - last_tick_data) > min_amp) { //enough amplitude?
      if (since_last_tick > min_tick_spacing) { //far enough from last tick?
        since_last_tick = 0;
        last_tick_data = data_shaft[1];
        if (hwp_data[0].direction) hwp_data[0].shaft_count++;
        else hwp_data[0].shaft_count--;
      }
    }
  }

  //write the count (converted to degrees) to the frame
  //(200 steps/rev) / (40 counts/rev) = (5 steps/count)
  degrees = Phytron_S2D(&bus, 0,
      hwp_data[0].shaft_count * 5 * bus.stepper[0].usteps);
  while (degrees < 0) degrees += 360.;
  while (degrees > 360.) degrees -= 360.;
  //TODO write more slowly once done testing
  WriteCalData(encCntHwpAddr[0], degrees, NIOS_QUEUE);
}

//create a thread for this function
void StartHWP(void)
{
  int all_ok = 0;
  int i;
  int j=0;
  int my_cindex = 0;
  int caddr_match = 0;
  int is_init = 0;
  int first_time=1;


  nameThread("HWP");
  bputs(startup, "Startup.");

  //Initialize global data
  for (i=0; i<NHWP; i++) {
    hwp_data[i].pos = 0;
    hwp_data[i].shaft_count = 0;
    hwp_data[i].direction = 1;
  }

  //Wait until in charge before doing any serial port stuff
  while (!InCharge) {
    if (first_time) {
      bprintf(info,"Not in charge.  Waiting.");
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
  while (!is_init) {
    if (first_time) {
      bprintf(info,"In Charge! Attempting to initalize.");
      first_time = 0;
    }
    if (Phytron_Init(&bus, HWP_BUS, "", HWP_CHATTER) == PH_ERR_OK)
      is_init = 1;
    usleep(10000);
    if (is_init) {
      bprintf(info,"Bus initialized on %ith attempt",j);
    }
    j++;
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
    caddr_match = 0;
    if (CommandData.hwp.caddr[my_cindex] >= 0) {
      bprintf(info, "Sending command \"%s\" to \"%s\"\n",
          CommandData.hwp.command[my_cindex], 
          bus.stepper[CommandData.hwp.caddr[my_cindex]].name);
      //increase print level for uplinked manual commands
      bus.chatter = PH_CHAT_BUS;
      if (CommandData.hwp.command[my_cindex][0] == '\\') {
        //'\\' indicates NAComm. Skip the '\\' character itself
        Phytron_NAComm(&bus, CommandData.hwp.caddr[my_cindex],
            CommandData.hwp.command[my_cindex] + 1);
      } else {
        Phytron_Comm(&bus, CommandData.hwp.caddr[my_cindex],
            CommandData.hwp.command[my_cindex]);
      }
      CommandData.hwp.caddr[my_cindex] = -1;
      bus.chatter = HWP_CHATTER;
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

