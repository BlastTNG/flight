/***************************************************************************
mcp: the BLAST master control program

This software is copyright (C) 2002-2006 University of Toronto

This file is part of mcp.

mcp is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
at your option) any later version.
 
mcp is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with mcp; if not, write to the Free Software Foundation, Inc.,
59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

**************************************************************************/


/*************************************************************************
 
 chrgctrl.c -- mcp code to read data from TriStar MPPT-60 charge
               controller over serial port using MODBUS comms. protocol.

*************************************************************************/

#include <stdio.h>           // standard input/output
#include <errno.h>
#include <sys/time.h>        // time structures for select()
#include <pthread.h>         // POSIX threads
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>          // POSIX symbolic constants
#include <termios.h>         // POSIX terminal control definitions
#include <fcntl.h>           // file control definitions
#include <modbus/modbus.h>

#include <channels_tng.h>
#include "chrgctrl.h"        // charge controller MODBUS comms
                             // function declarations
#include "mcp.h"

typedef enum {
    CC_STATE_DISCONNECT = 0,
    CC_STATE_READY,
    CC_STATE_RESET,
    CC_STATE_SHUTDOWN
} e_cc_state;

/* charge controller data struct
   written to by serial thread in chrgctrl.c */

typedef struct {
    int id;                      // Which Charge controller are we?
    char *addr;                  // IP Address of Charge controller
    pthread_t tid;               // Thread ID of CC thread
    modbus_t *mb;

    int state;
    int req_state;
    int has_error;

    double V_batt;               // battery voltage from sense terminals
    double V_arr;                // solar array input voltage
    double I_batt;               // current to battery
    double I_arr;                // current from solar array (+/- 20%)
    double V_targ;               // target charging voltage

    int T_hs;                    // heatsink temperature

    uint16_t fault_field;  // fault bitfield
    uint32_t alarm_field; // alarm bitfield
    unsigned int led_state;      // state of status LEDs on front of unit
    unsigned int charge_state;   // charging state of controller
} charge_ctl_t;

static charge_ctl_t charge_controller[2] = {{.id = 1,
                                             .addr = "192.168.1.253"},
                                            {.id = 2,
                                             .addr = "192.168.1.252"}};

static void* chrgctrlComm(void* cc);

void nameThread(const char*);	      // in mcp.c

void store_charge_controller_data(void)
{
  static channel_t *VBattCCAddr[2];
  static channel_t *VArrCCAddr[2];
  static channel_t *IBattCCAddr[2];
  static channel_t *IArrCCAddr[2];
  static channel_t *VTargCCAddr[2];
  static channel_t *ThsCCAddr[2];
  static channel_t *FaultCCAddr[2];
  static channel_t *AlarmCCAddr[2];
  static channel_t *ChargeCCAddr[2];
  static channel_t *LEDCCAddr[2];

  static int firsttime = 1;

  if (firsttime) {
    firsttime = 0;

    VBattCCAddr[0] = channels_find_by_name("v_batt_cc1");
    VArrCCAddr[0] = channels_find_by_name("v_arr_cc1");
    IBattCCAddr[0] = channels_find_by_name("i_batt_cc1");
    IArrCCAddr[0]  = channels_find_by_name("i_arr_cc1");
    VTargCCAddr[0] = channels_find_by_name("v_targ_cc1");
    ThsCCAddr[0] = channels_find_by_name("t_hs_cc1");
    FaultCCAddr[0] = channels_find_by_name("fault_cc1");
    AlarmCCAddr[0] = channels_find_by_name("alarm_cc1");
    ChargeCCAddr[0] = channels_find_by_name("state_cc1");
    LEDCCAddr[0] = channels_find_by_name("led_cc1");

    VBattCCAddr[1] = channels_find_by_name("v_batt_cc2");
    VArrCCAddr[1] = channels_find_by_name("v_arr_cc2");
    IBattCCAddr[1] = channels_find_by_name("i_batt_cc2");
    IArrCCAddr[1]  = channels_find_by_name("i_arr_cc2");
    VTargCCAddr[1] = channels_find_by_name("v_targ_cc2");
    ThsCCAddr[1] = channels_find_by_name("t_hs_cc2");
    FaultCCAddr[1] = channels_find_by_name("fault_cc2");
    AlarmCCAddr[1] = channels_find_by_name("alarm_cc2");
    ChargeCCAddr[1] = channels_find_by_name("state_cc2");
    LEDCCAddr[1] = channels_find_by_name("led_cc2");
  }

  for (int i = 0; i < 2; i++) {
      SET_SCALED_VALUE(VBattCCAddr[i], charge_controller[i].V_batt);
      SET_SCALED_VALUE(VArrCCAddr[i], charge_controller[i].V_arr);
      SET_SCALED_VALUE(IBattCCAddr[i], charge_controller[i].I_batt);
      SET_SCALED_VALUE(IArrCCAddr[i],  charge_controller[i].I_arr);
      SET_SCALED_VALUE(VTargCCAddr[i], charge_controller[i].V_targ);
      SET_VALUE(ThsCCAddr[i], charge_controller[i].T_hs);
      SET_VALUE(FaultCCAddr[i], charge_controller[i].fault_field);
      SET_VALUE(AlarmCCAddr[i], charge_controller[i].alarm_field);
      SET_VALUE(ChargeCCAddr[i], charge_controller[i].charge_state);
      SET_VALUE(LEDCCAddr[i], charge_controller[i].led_state);
  }
}

/* create charge controller serial thread */

void startChrgCtrl(int m_controller)
{
    blast_info("startChrgCtrl: creating charge controller %d ModBus thread", m_controller);
    charge_controller[m_controller].req_state = CC_STATE_READY;

    pthread_create(&charge_controller[m_controller].tid, NULL, chrgctrlComm, &charge_controller[m_controller]);
}

/* thread routine: continously poll charge controller for data */

void* chrgctrlComm(void* cc) {
    static int have_warned_connect = 0;
    charge_ctl_t *ctlr = (charge_ctl_t*)cc;
    char tname[10];
    float Vscale, Iscale;
    uint16_t tmp_data[255];

    snprintf(tname, sizeof(tname), "ChrgC%1d", ctlr->id);
    nameThread(tname);

    blast_info("starting controller #%d at IP %s", ctlr->id, ctlr->addr);

    while (ctlr->req_state != CC_STATE_SHUTDOWN) {
        usleep(1000000);
        if (ctlr->state == CC_STATE_RESET ||
                (ctlr->req_state == CC_STATE_READY
                  && ctlr->state != CC_STATE_READY)) {
            if (ctlr->mb) {
                modbus_close(ctlr->mb);
                modbus_free(ctlr->mb);
            }
            ctlr->mb = modbus_new_tcp(ctlr->addr, 502);

            struct timeval tv;
            tv.tv_sec = 1;
            tv.tv_usec = 0;
            modbus_set_slave(ctlr->mb, 1);
            modbus_set_response_timeout(ctlr->mb, &tv);

            if (modbus_connect(ctlr->mb)) {
                if (!have_warned_connect) {
                    blast_err("Could not connect to ModBUS charge controller at %s: %s", ctlr->addr,
                            modbus_strerror(errno));
                }
                modbus_free(ctlr->mb);
                ctlr->mb = NULL;
                ctlr->state = CC_STATE_DISCONNECT;
                have_warned_connect = 1;
                continue;
            }
            ctlr->state = CC_STATE_READY;
            have_warned_connect = 0;

            /**
             * Read the scaling factors immediately after reconnecting
             */
            if (modbus_read_registers(ctlr->mb, 0, 4, tmp_data) < 0) {
                blast_err("Could not read scaling factors for Modbus"
                        " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
                continue;
            } else {
                Vscale = tmp_data[0] + tmp_data[1] / 65536.0;
                Iscale = tmp_data[2] + tmp_data[3] / 65536.0;
            }
        }

        if (ctlr->state != CC_STATE_READY) {
            blast_info("Charge controller %d not ready!  Sleeping...", ctlr->id);
            continue;
        }
        ctlr->has_error = 0;
        /**
         * Addressing of the charge controller references the PDU Addr
         */

        if (modbus_read_registers(ctlr->mb, 0x0018, 11, tmp_data) < 0) {
            blast_err("Could not read voltages for Modbus"
                    " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
            ctlr->has_error = 1;
            ctlr->V_batt = 0;
            ctlr->V_arr = 0;
            ctlr->I_batt = 0;
            ctlr->I_arr = 0;
        } else {
            ctlr->V_batt = ((int16_t)tmp_data[1]) * Vscale / 32768.0;
            ctlr->V_arr = ((int16_t)tmp_data[3]) * Vscale / 32768.0;
            ctlr->I_batt = ((int16_t)tmp_data[4]) * Iscale / 32768.0;
            ctlr->I_arr = ((int16_t)tmp_data[5]) * Iscale / 32768.0;
        }

        /* heatsink temperature in degrees C (addr 0x0023) */

        if (modbus_read_registers(ctlr->mb, 0x0023, 1, tmp_data) < 0) {
            blast_err("Could not read heatsink temp for Modbus"
                    " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
            ctlr->has_error = 1;
            ctlr->T_hs = 0;
        } else {
            ctlr->T_hs = tmp_data[0];
        }

        /* charge controller fault bitfield (addr 0x002c) */

        if (modbus_read_registers(ctlr->mb, 0x002C, 1, tmp_data) < 0) {
            blast_err("Could not read fault bitfield for Modbus"
                    " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
            ctlr->has_error = 1;
            ctlr->fault_field = 0;
        } else {
            ctlr->fault_field = tmp_data[0];
        }

        /* charge controller alarm bitfield (spans 2 regs with addrs 47,48) */

        if (modbus_read_registers(ctlr->mb, 0x002E, 2, tmp_data) < 0) {
            blast_err("Could not read alarm bitfield for Modbus"
                    " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
            ctlr->has_error = 1;
            ctlr->alarm_field = 0;
        } else {
            ctlr->alarm_field = tmp_data[0] << 16 | tmp_data[1];
        }

        /* controller LED state, charge state and target charging voltage (addrs 50, 51, 52) */

        if (modbus_read_registers(ctlr->mb, 0x0031, 3, tmp_data) < 0) {
            blast_err("Could not read state vars for Modbus"
                    " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
            ctlr->has_error = 1;
            ctlr->V_targ = 0.0;
            ctlr->charge_state = 10;
            ctlr->led_state = 18;
        } else {
            ctlr->led_state = tmp_data[0];
            ctlr->charge_state = tmp_data[1];
            ctlr->V_targ = ((int16_t)tmp_data[2]) * Vscale / 32768.0;
        }


//        blast_dbg("Voltage Scaling Factor: %.3f \n", Vscale);
//        blast_dbg("Current Scaling Factor: %.3f \n \n", Iscale);
//        blast_dbg("Battery Sense Voltage: %.3f V \n", ctlr->V_batt);
//        blast_dbg("Solar Array Input Voltage: %.3f V \n", ctlr->V_arr);
//        blast_dbg("Output Current to Battery: %.3f A \n", ctlr->I_batt);
//        blast_dbg("Input Current from Array: %.3f A \n", ctlr->I_arr);
//        blast_dbg("Target Battery Charging Voltage: %.3f V \n \n", ctlr->V_targ);
//        blast_dbg("Heatsink Temperature: %d C \n \n", ctlr->T_hs);
    }

    if (ctlr->mb) {
        modbus_close(ctlr->mb);
        modbus_free(ctlr->mb);
    }
    return NULL;
}
