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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <float.h>
#include <sys/time.h>

#include <conversions.h>
#include <blast_sip_interface.h>

#include "channels_tng.h"
#include "pointing_struct.h"
#include "tx.h"
#include "command_struct.h"
#include "mcp.h"
#include "chrgctrl.h"
#include "flcdataswap.h"

#include <motors.h>

#define NIOS_BUFFER_SIZE 100

extern short int SouthIAm;

extern int StartupVeto;

extern short int InCharge;

int EthernetIsc = 3;
int EthernetOsc = 3;
int EthernetSBSC = 3;

extern struct chat_buf chatter_buffer;  /* mcp.c */

/* in auxiliary.c */
void ChargeController(void);
void ControlAuxMotors();
void ControlGyroHeat();
void CameraTrigger(int which);
void ControlPower(void);
void VideoTx(void);

/* in das.c */
void BiasControl();
void CryoControl(int index);
void PhaseControl(void);

/* in sbsc.cpp */
void cameraFields();        

/* in sched.c */
void DoSched();

/* in xystage.c */
void StoreStageBus(int index);

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
    static channel_t* rateTdrssAddr;
    static channel_t* rateIridiumAddr;
    static channel_t* tChipFlcAddr;
    static channel_t* tMbFlcAddr;
    static channel_t* statusMCCAddr;
    static channel_t* ploverAddr;
    static channel_t* he4LevOldAddr;
    static channel_t* statusEthAddr;
    static channel_t* he4LevReadAddr;
    static channel_t* partsSchedAddr;
    static channel_t* upslotSchedAddr;

    static channel_t* timeMeFlcAddr;
    static channel_t* dfMeFlcAddr;
    static channel_t* timeoutMeAddr;
    static channel_t* tCpuMeFlcAddr;
    static channel_t* lastMeCmdAddr;
    static channel_t* countMeCmdAddr;

    static channel_t* timeOtherFlcAddr;
    static channel_t* dfOtherFlcAddr;
    static channel_t* timeoutOtherAddr;
    static channel_t* tCpuOtherFlcAddr;
    static channel_t* lastOtherCmdAddr;
    static channel_t* countOtherCmdAddr;

    struct flc_data otherData, *myData;

    static int incharge = -1;
    time_t t;
    int i_point;
    struct timeval tv;
    struct timezone tz;

    unsigned short mccstatus;

    static int firsttime = 1;
    if (firsttime) {
        char buf[128];
        firsttime = 0;
        statusMCCAddr = channels_find_by_name("status_mcc");

        he4LevOldAddr = channels_find_by_name("he4_lev_old");
        he4LevReadAddr = channels_find_by_name("he4_lev");

        tChipFlcAddr = channels_find_by_name("t_chip_flc");
        tMbFlcAddr = channels_find_by_name("t_mb_flc");
        timeAddr = channels_find_by_name("time");
        timeUSecAddr = channels_find_by_name("time_usec");
        rateTdrssAddr = channels_find_by_name("rate_tdrss");
        rateIridiumAddr = channels_find_by_name("rate_iridium");

        ploverAddr = channels_find_by_name("plover");
        statusEthAddr = channels_find_by_name("status_eth");
        partsSchedAddr = channels_find_by_name("parts_sched");
        upslotSchedAddr = channels_find_by_name("upslot_sched");

        sprintf(buf, "time_%c_flc", (SouthIAm) ? 's' : 'n');
        timeMeFlcAddr = channels_find_by_name(buf);
        sprintf(buf, "df_%c_flc", (SouthIAm) ? 's' : 'n');
        dfMeFlcAddr = channels_find_by_name(buf);
        sprintf(buf, "timeout_%c", (SouthIAm) ? 's' : 'n');
        timeoutMeAddr = channels_find_by_name(buf);
        sprintf(buf, "t_cpu_%c_flc", (SouthIAm) ? 's' : 'n');
        tCpuMeFlcAddr = channels_find_by_name(buf);
        sprintf(buf, "last_%c_cmd", (SouthIAm) ? 's' : 'n');
        lastMeCmdAddr = channels_find_by_name(buf);
        sprintf(buf, "count_%c_cmd", (SouthIAm) ? 's' : 'n');
        countMeCmdAddr = channels_find_by_name(buf);

        sprintf(buf, "time_%c_flc", (!SouthIAm) ? 's' : 'n');
        timeOtherFlcAddr = channels_find_by_name(buf);
        sprintf(buf, "df_%c_flc", (!SouthIAm) ? 's' : 'n');
        dfOtherFlcAddr = channels_find_by_name(buf);
        sprintf(buf, "timeout_%c", (!SouthIAm) ? 's' : 'n');
        timeoutOtherAddr = channels_find_by_name(buf);
        sprintf(buf, "t_cpu_%c_flc", (!SouthIAm) ? 's' : 'n');
        tCpuOtherFlcAddr = channels_find_by_name(buf);
        sprintf(buf, "last_%c_cmd", (!SouthIAm) ? 's' : 'n');
        lastOtherCmdAddr = channels_find_by_name(buf);
        sprintf(buf, "count_%c_cmd", (!SouthIAm) ? 's' : 'n');
        countOtherCmdAddr = channels_find_by_name(buf);
    }

    myData = get_flc_out_data();

    if (StartupVeto > 0) {
        InCharge = 0;
    }
    else {
        InCharge = !(SouthIAm ^ (GET_UINT16(statusMCCAddr) & 0x1));
    }
    if (InCharge != incharge && InCharge) {
        blast_info("System: I, %s, have gained control.\n", SouthIAm ? "South" : "North");
        CommandData.actbus.force_repoll = 1;
    }
    else if (InCharge != incharge) {
        blast_info("System: I, %s, have lost control.\n", SouthIAm ? "South" : "North");
    }

    if (CommandData.Cryo.heliumLevel)
        GET_VALUE(he4LevReadAddr, CommandData.Cryo.heliumLevel);

    SET_VALUE(he4LevOldAddr, CommandData.Cryo.he4_lev_old);

    incharge = InCharge;

    gettimeofday(&tv, &tz);

    SET_VALUE(timeAddr, tv.tv_sec + TEMPORAL_OFFSET);
    SET_VALUE(timeUSecAddr, tv.tv_usec);

    SET_VALUE(timeMeFlcAddr, tv.tv_sec + TEMPORAL_OFFSET);
    myData->time = tv.tv_sec + TEMPORAL_OFFSET;

    SET_VALUE(tChipFlcAddr, CommandData.temp1);
    SET_VALUE(tMbFlcAddr, CommandData.temp3);
    SET_VALUE(tCpuMeFlcAddr, CommandData.temp2);
    myData->t_cpu = CommandData.temp2;

    SET_VALUE(dfMeFlcAddr, CommandData.df);
    myData->df = CommandData.df;

    SET_VALUE(partsSchedAddr, CommandData.parts_sched & 0xffffff);
    SET_VALUE(upslotSchedAddr, CommandData.upslot_sched);

    i_point = GETREADINDEX(point_index);

#ifdef BOLOTEST
    t = mcp_systime(NULL);
#else
    t = PointingData[i_point].t;
#endif

    if (CommandData.pointing_mode.t > t) {
        SET_VALUE(timeoutMeAddr, CommandData.pointing_mode.t - t);
        myData->timeout = CommandData.pointing_mode.t - t;
    }
    else {
        SET_VALUE(timeoutMeAddr, 0);
        myData->timeout = 0;
    }

    SET_VALUE(ploverAddr, CommandData.plover);
    SET_VALUE(rateTdrssAddr, CommandData.tdrss_bw);
    SET_VALUE(rateIridiumAddr, CommandData.iridium_bw);

    SET_VALUE(statusEthAddr, //first two bits used to be sun sensor
    ((EthernetIsc & 0x3) << 2) + ((EthernetOsc & 0x3) << 4) + ((EthernetSBSC & 0x3) << 6));

    mccstatus = (SouthIAm ? 0x1 : 0x0) +                 //0x01
            (CommandData.at_float ? 0x2 : 0x0) +     //0x02
            (CommandData.uplink_sched ? 0x08 : 0x00) + //0x08
            (CommandData.sucks ? 0x10 : 0x00) +      //0x10
            //((CommandData.lat_range & 0x3) << 5) +   //0x60
            ((CommandData.slot_sched & 0xFF) << 8);  //0xFF00

    if (CommandData.uplink_sched) {
        mccstatus |= 0x60;
    }
    else {
        mccstatus |= ((CommandData.lat_range & 0x3) << 5);
    }

    SET_VALUE(statusMCCAddr, mccstatus);

    SET_VALUE(lastMeCmdAddr, CommandData.last_command);
    SET_VALUE(countMeCmdAddr, CommandData.command_count);
    myData->last_command = CommandData.last_command;
    myData->command_count = CommandData.command_count;

    swap_flc_data(&otherData);
    SET_VALUE(timeOtherFlcAddr, otherData.time);
    SET_VALUE(dfOtherFlcAddr, otherData.df);
    SET_VALUE(timeoutOtherAddr, otherData.timeout);
    SET_VALUE(tCpuOtherFlcAddr, otherData.t_cpu);
    SET_VALUE(lastOtherCmdAddr, otherData.last_command);
    SET_VALUE(countOtherCmdAddr, otherData.command_count);
}

void WriteChatter(void)
{
//    static channel_t* chatterAddr;
//    static int firsttime = 1;
//    unsigned int chat;
//
//    if (firsttime) {
//        firsttime = 0;
//        chatterAddr = channels_find_by_name("chatter");
//    }
//
//    switch (index & 0x03) {
//        case 0x00:
//            chat = 0x0000;
//            break;
//        case 0x01:
//            chat = 0x8000;
//            break;
//        case 0x02:
//            chat = 0x0080;
//            break;
//        case 0x03:
//            chat = 0x8080;
//            break;
//        default:
//            chat = 0x0000;
//            break;
//    }
//
//    chat += (unsigned int) (chatter_buffer.msg[chatter_buffer.reading][index * 2] & 0x7F);
//    chat += (unsigned int) ((chatter_buffer.msg[chatter_buffer.reading][(index * 2) + 1]) & 0x7F) << 8;
//
//    if (index == (19)) {
//        chatter_buffer.reading = (chatter_buffer.reading + 1) & 0x3;
//    }
//
//    SET_VALUE(chatterAddr, chat);
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
    unsigned int GyroMask = 0x3f; //all gyros enabled
    int convert[6] = { 1, 5, 0, 2, 3, 4 }; //order of gyros in power switching
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
        }
        else if ((CommandData.gymask & (0x01 << i)) == 0) {
            GyroMask &= ~(0x01 << i);
        }
        else {
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


