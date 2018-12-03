/* mcp: the master control program
 *
 * tx.c writes data from mcp to the nios (bbc) to include it in frames
 * 
 * This software is copyright (C) 2002-2011 University of Toronto
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

/* tx.c */

/* NB: As of 7 Sep 2003 this file has been split up into four pieces:
 *
 * auxiliary.c: Auxiliary controls: Lock Motor, Pumps, Electronics Heat, ISC
 * das.c:       DAS, Bias and Cryo controls
 * motors.c:    Motor commanding and Scan modes
 * tx.c:        Pointing data writeback, ADC sync, and standard Tx frame control
 *
 * -dvw */

#include "tx.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <float.h>
#include <sys/time.h>

#include <conversions.h>
#include <computer_sensors.h>

#include "channels_tng.h"
#include "pointing_struct.h"
#include "command_struct.h"
#include "mcp.h"
#include "chrgctrl.h"

#include "motors.h"

#define NIOS_BUFFER_SIZE 100

extern int16_t SouthIAm;

extern int16_t InCharge;

int EthernetIsc = 3;
int EthernetOsc = 3;
int EthernetSBSC = 3;

/* in auxiliary.c */
void ChargeController(void);
void ControlAuxMotors();
void VideoTx(void);

/* in das.c */
void BiasControl();
void CryoControl(int index);
void PhaseControl(void);

/* in sched.c */
void DoSched();

/* in xystage.c */
// void StoreStageBus(int index);

/* this is provided to let the various controls know that we're doing our
 * initial control writes -- there's no input data yet */
int mcp_initial_controls = 0;

/************************************************************************/
/*                                                                      */
/*  WriteAux: write aux data, like cpu time, temperature                */
/*                                                                      */
/************************************************************************/
void WriteAux(void)
{
    static channel_t* timeAddr;
    static channel_t* timeUSecAddr;
    static channel_t* rateHighrateAddr;
    static channel_t* rateBiphaseAddr;
    static channel_t* ratePilotAddr;
    static channel_t* rateMPSSEClockAddr;

    static channel_t* statusMCCAddr;
    static channel_t* ploverAddr;
    static channel_t* he4LevOldAddr;
    static channel_t* statusEthAddr;
    static channel_t* he4LevReadAddr;
    static channel_t* partsSchedAddr;
    static channel_t* upslotSchedAddr;

    static channel_t* tcpu0_flc_addr[2];
    static channel_t* tcpu1_flc_addr[2];
    static channel_t* v12_flc_addr[2];
    static channel_t* v5_flc_addr[2];
    static channel_t* vbatt_flc_addr[2];
    static channel_t* icurr_flc_addr[2];

    static channel_t* time_flc_addr[2];
    static channel_t* df_flc_addr[2];
    static channel_t* timeout_addr[2];
    static channel_t* last_cmd_addr[2];
    static channel_t* count_cmd_addr[2];

    const char which_flc[2] = {'n', 's'};

#define ASSIGN_BOTH_FLC(_ch, _str) \
    ({ \
        char buf[128];\
        snprintf(buf, sizeof(buf),  _str  "_%c", which_flc[SouthIAm]); \
        _ch[0] = channels_find_by_name(buf); \
        snprintf(buf, sizeof(buf), _str  "_%c", which_flc[!SouthIAm]); \
        _ch[1] = channels_find_by_name(buf); \
    })

    time_t t;
    int i_point;
    struct timeval tv;
    struct timezone tz;

    uint16_t  mccstatus;

    static int firsttime = 1;
    if (firsttime) {
        firsttime = 0;

        // status and housekeeping information
        statusMCCAddr = channels_find_by_name("status_mcc");

        he4LevOldAddr = channels_find_by_name("he4_lev_old");
        he4LevReadAddr = channels_find_by_name("he4_lev");

        timeAddr = channels_find_by_name("time");
        timeUSecAddr = channels_find_by_name("time_usec");
        rateHighrateAddr = channels_find_by_name("rate_highrate");
        rateBiphaseAddr = channels_find_by_name("rate_biphase");
        ratePilotAddr = channels_find_by_name("rate_pilot");
        rateMPSSEClockAddr = channels_find_by_name("mpsse_clock_speed");

        ploverAddr = channels_find_by_name("plover");
        statusEthAddr = channels_find_by_name("status_eth");
        partsSchedAddr = channels_find_by_name("parts_sched");
        upslotSchedAddr = channels_find_by_name("upslot_sched");

        // housekeeping info to be shared with other flight computer
        // North/South specific (denoted ***_n or ***_s)
        // names are automatically aliased with the correct computer
        ASSIGN_BOTH_FLC(tcpu0_flc_addr, "t_cpu0_flc");
        ASSIGN_BOTH_FLC(tcpu1_flc_addr, "t_cpu1_flc");
        ASSIGN_BOTH_FLC(v12_flc_addr, "v_12v_flc");
        ASSIGN_BOTH_FLC(v5_flc_addr, "v_5v_flc");
        ASSIGN_BOTH_FLC(vbatt_flc_addr, "v_batt_flc");
        ASSIGN_BOTH_FLC(icurr_flc_addr, "i_flc");

        ASSIGN_BOTH_FLC(last_cmd_addr, "last_cmd");
        ASSIGN_BOTH_FLC(count_cmd_addr, "count_cmd");
        ASSIGN_BOTH_FLC(df_flc_addr, "df_flc");
        ASSIGN_BOTH_FLC(time_flc_addr, "time_flc");
        ASSIGN_BOTH_FLC(timeout_addr, "timeout");
    }

    gettimeofday(&tv, &tz);

    SET_VALUE(timeAddr, tv.tv_sec + TEMPORAL_OFFSET);
    SET_VALUE(timeUSecAddr, tv.tv_usec);
    SET_VALUE(time_flc_addr[0], tv.tv_sec + TEMPORAL_OFFSET);
    SET_VALUE(tcpu0_flc_addr[0], computer_sensors.core0_temp);
    SET_VALUE(tcpu1_flc_addr[0], computer_sensors.core1_temp);
    SET_VALUE(icurr_flc_addr[0], computer_sensors.curr_input);
    SET_VALUE(v12_flc_addr[0], computer_sensors.volt_12V);
    SET_VALUE(v5_flc_addr[0], computer_sensors.volt_5V);
    SET_VALUE(vbatt_flc_addr[0], computer_sensors.volt_battery);
    SET_VALUE(df_flc_addr[0], computer_sensors.disk_free);
    SET_VALUE(partsSchedAddr, CommandData.parts_sched & 0xffffff);
    SET_VALUE(upslotSchedAddr, CommandData.upslot_sched);
    i_point = GETREADINDEX(point_index);

#ifdef BOLOTEST
    t = mcp_systime(NULL);
#else
    t = PointingData[i_point].t;
#endif

    if (CommandData.pointing_mode.t > t) {
        SET_VALUE(timeout_addr[0], CommandData.pointing_mode.t - t);
    } else {
        SET_VALUE(timeout_addr[0], 0);
    }

    SET_VALUE(ploverAddr, CommandData.plover);
    SET_VALUE(rateHighrateAddr, CommandData.highrate_bw*8.0/1000.0); // Bps
    SET_VALUE(rateBiphaseAddr, CommandData.biphase_bw*8.0/1000.0); // Bps
    SET_VALUE(ratePilotAddr, CommandData.pilot_bw*8.0/1000.0); // Bps

    SET_VALUE(rateMPSSEClockAddr, CommandData.biphase_clk_speed/1000.0);

    SET_VALUE(statusEthAddr, // first two bits used to be sun sensor
    ((EthernetIsc & 0x3) << 2) + ((EthernetOsc & 0x3) << 4) + ((EthernetSBSC & 0x3) << 6));

    mccstatus = (SouthIAm ? 0x1 : 0x0) +                 // 0x01
            (CommandData.at_float ? 0x2 : 0x0) +     // 0x02
            (CommandData.uplink_sched ? 0x08 : 0x00) + // 0x08
            (CommandData.sucks ? 0x10 : 0x00) +      // 0x10
            (InCharge ? 0x80 : 0x00) +               // 0x80
//            ((CommandData.lat_range & 0x3) << 5) +   // 0x60
            ((CommandData.slot_sched & 0xFF) << 8);  // 0xFF00

    if (CommandData.uplink_sched) {
        mccstatus |= 0x60;
    } else {
        mccstatus |= ((CommandData.lat_range & 0x3) << 5);
    }

    SET_VALUE(statusMCCAddr, mccstatus);
    SET_VALUE(last_cmd_addr[0], CommandData.last_command);
    SET_VALUE(count_cmd_addr[0], CommandData.command_count);
}

/***************************************************************/
/* SetGyroMask:                                                */
/* mask gyros - automatic masking (if the gyro is faulty)      */
/*           or- commanded masking                             */
/* power cycle gyros - if masked for 1s                        */
/*                and- hasn't been power cycled in the last 25s */
/***************************************************************/
#define MASK_TIMEOUT 5 /* 1 sec -- in 5Hz Slow Frames */
#define GYRO_PCYCLE_TIMEOUT 125 /* 25 sec */
void SetGyroMask(void)
{
    static channel_t* maskGyAddr;
    unsigned int GyroMask = 0x3f; // all gyros enabled
    int convert[6] = { 1, 5, 0, 2, 3, 4 }; // order of gyros in power switching
    static int t_mask[6] = { 0, 0, 0, 0, 0, 0 };
    static int wait[6] = { 0, 0, 0, 0, 0, 0 };
    static channel_t* faultGyAddr;
    static int firsttime = 1;
    unsigned int GyroFault;

    if (firsttime) {
        firsttime = 0;
        maskGyAddr = channels_find_by_name("mask_gy");
        faultGyAddr = channels_find_by_name("fault_gy");
    }

    GyroFault = GET_UINT16(faultGyAddr);
    int i;
    for (i = 0; i < 6; i++) {
        int j = convert[i];
        if (GyroFault & (0x01 << i)) {
            GyroMask &= ~(0x01 << i);
            t_mask[i] += 1;
            if (t_mask[i] > MASK_TIMEOUT) {
                if (wait[i] == 0) {
                    CommandData.power.gyro_off_auto[j] = PCYCLE_HOLD_LEN;
                    wait[i] = GYRO_PCYCLE_TIMEOUT;
                }
            }
        } else if ((CommandData.gymask & (0x01 << i)) == 0) {
            GyroMask &= ~(0x01 << i);
        } else {
            GyroMask |= 0x01 << i;
            t_mask[i] = 0;
        }

        if (wait[i] > 0)
            wait[i]--;
    }

    SET_UINT16(maskGyAddr, GyroMask);
}



#ifndef BOLOTEST


#endif


double ReadCalData(channel_t *m_ch)
{
    double retval = NAN;

    GET_VALUE(m_ch, retval);
    return (retval * m_ch->m_c2e + m_ch->b_e2e);
}
