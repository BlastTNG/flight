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
#define CHRGCTRL1_DEVICE "/dev/ttySI3" // change depending upon serial hub port
#define CHRGCTRL2_DEVICE "/dev/ttySI6" // change depending upon serial hub port
#define QUERY_SIZE 6
#define CHECKSUM_SIZE 2


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
    unsigned int alarm_field_hi; // alarm high bitfield
    unsigned int alarm_field_lo; // alarm low bitfield
    unsigned int led_state;      // state of status LEDs on front of unit
    unsigned int charge_state;   // charging state of controller
} charge_ctl_t;

static charge_ctl_t charge_controller[2] = {{.id = 1,
                                             .addr = "192.168.1.253"},
                                            {.id = 2,
                                             .addr = "192.168.1.252"}};

static void* chrgctrlComm(void* cc);

void nameThread(const char*);	      // in mcp.c
extern int16_t InCharge;            // in tx.c

void store_charge_controller_data(void)
{
  static channel_t *VBattCC1Addr;
  static channel_t *VArrCC1Addr;
  static channel_t *IBattCC1Addr;
  static channel_t *IArrCC1Addr;
  static channel_t *VTargCC1Addr;
  static channel_t *ThsCC1Addr;
  static channel_t *FaultCC1Addr;
  static channel_t *AlarmHiCC1Addr;
  static channel_t *AlarmLoCC1Addr;
  static channel_t *ChargeCC1Addr;
  static channel_t *LEDCC1Addr;

  static channel_t *VBattCC2Addr;
  static channel_t *VArrCC2Addr;
  static channel_t *IBattCC2Addr;
  static channel_t *IArrCC2Addr;
  static channel_t *VTargCC2Addr;
  static channel_t *ThsCC2Addr;
  static channel_t *FaultCC2Addr;
  static channel_t *AlarmHiCC2Addr;
  static channel_t *AlarmLoCC2Addr;
  static channel_t *ChargeCC2Addr;
  static channel_t *LEDCC2Addr;

  static int firsttime = 1;

  if (firsttime) {
    firsttime = 0;

    VBattCC1Addr = channels_find_by_name("v_batt_cc1");
    VArrCC1Addr = channels_find_by_name("v_arr_cc1");
    IBattCC1Addr = channels_find_by_name("i_batt_cc1");
    IArrCC1Addr  = channels_find_by_name("i_arr_cc1");
    VTargCC1Addr = channels_find_by_name("v_targ_cc1");
    ThsCC1Addr = channels_find_by_name("t_hs_cc1");
    FaultCC1Addr = channels_find_by_name("fault_cc1");
    AlarmHiCC1Addr = channels_find_by_name("alarm_hi_cc1");
    AlarmLoCC1Addr = channels_find_by_name("alarm_lo_cc1");
    ChargeCC1Addr = channels_find_by_name("state_cc1");
    LEDCC1Addr = channels_find_by_name("led_cc1");

    VBattCC2Addr = channels_find_by_name("v_batt_cc2");
    VArrCC2Addr = channels_find_by_name("v_arr_cc2");
    IBattCC2Addr = channels_find_by_name("i_batt_cc2");
    IArrCC2Addr  = channels_find_by_name("i_arr_cc2");
    VTargCC2Addr = channels_find_by_name("v_targ_cc2");
    ThsCC2Addr = channels_find_by_name("t_hs_cc2");
    FaultCC2Addr = channels_find_by_name("fault_cc2");
    AlarmHiCC2Addr = channels_find_by_name("alarm_hi_cc2");
    AlarmLoCC2Addr = channels_find_by_name("alarm_lo_cc2");
    ChargeCC2Addr = channels_find_by_name("state_cc2");
    LEDCC2Addr = channels_find_by_name("led_cc2");
  }

  SET_VALUE(VBattCC1Addr, 180.0*charge_controller[0].V_batt + 32400.0);
  SET_VALUE(VArrCC1Addr, 180.0*charge_controller[0].V_arr + 32400.0);
  SET_VALUE(IBattCC1Addr, 400.0*charge_controller[0].I_batt + 32000.0);
  SET_VALUE(IArrCC1Addr,  400.0*charge_controller[0].I_arr + 32000.0);
  SET_VALUE(VTargCC1Addr, 180.0*charge_controller[0].V_targ + 32400.0);
  SET_VALUE(ThsCC1Addr, charge_controller[0].T_hs);
  SET_VALUE(FaultCC1Addr, charge_controller[0].fault_field);
  SET_VALUE(AlarmHiCC1Addr, charge_controller[0].alarm_field_hi);
  SET_VALUE(AlarmLoCC1Addr, charge_controller[0].alarm_field_lo);
  SET_VALUE(ChargeCC1Addr, charge_controller[0].charge_state);
  SET_VALUE(LEDCC1Addr, charge_controller[0].led_state);

  SET_VALUE(VBattCC2Addr, 180.0*charge_controller[1].V_batt + 32400.0);
  SET_VALUE(VArrCC2Addr, 180.0*charge_controller[1].V_arr + 32400.0);
  SET_VALUE(IBattCC2Addr, 400.0*charge_controller[1].I_batt + 32000.0);
  SET_VALUE(IArrCC2Addr,  400.0*charge_controller[1].I_arr + 32000.0);
  SET_VALUE(VTargCC2Addr, 180.0*charge_controller[1].V_targ + 32400.0);
  SET_VALUE(ThsCC2Addr, charge_controller[1].T_hs);
  SET_VALUE(FaultCC2Addr, charge_controller[1].fault_field);
  SET_VALUE(AlarmHiCC2Addr, charge_controller[1].alarm_field_hi);
  SET_VALUE(AlarmLoCC2Addr, charge_controller[1].alarm_field_lo);
  SET_VALUE(ChargeCC2Addr, charge_controller[1].charge_state);
  SET_VALUE(LEDCC2Addr, charge_controller[1].led_state);
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
    snprintf(tname, sizeof(tname), "ChrgC%1d", ctlr->id);
    nameThread(tname);

    blast_info("starting controller #%d at IP %s", ctlr->id, ctlr->addr);

    /* declare one chrgctrl_rawdata struct for each contiguous set of registers
     to be read */

    struct chrgctrl_rawdata {
        int num;                // # of data bytes returned by controller
        uint16_t arr[MAX_READ_REGS]; // array to store serial data read
    } scale, elec, temp, fault, alarm, charge;

    /*

     This is a relic from the test program. Not needed for mcp
     since bitfields are in derived.c

     struct status fault_bits[] = {

     {1, "overcurrent"},
     {2, "FETs shorted"},
     {4, "software bug"},
     {8, "battery high voltage disconnect"},
     {16, "array high voltage disconnect"},
     {32, "settings switch changed"},
     {64, "custom settings edit"},
     {128, "remote temp. sensor shorted"},
     {256, "remote temp. sensor disconnected"},
     {512, "EEPROM retry limit"},
     {1024, "slave control timeout"}
     };

     struct status alarm_bits[] = {

     {1, "remote temp. sensor open"},
     {2, "remote temp. sensor shorted"},
     {4, "remote temp. sensor disconnected"},
     {8, "heatsink temp. sensor open"},
     {16, "heatsink temp. sensor shorted"},
     {32, "high temp. current limit"},
     {64, "current limit"},
     {128, "current offset"},
     {256, "battery sense out of range"},
     {512, "battery sense disconnected"},
     {1024, "uncalibrated"},
     {2048, "remote temp. sensor miswire"},
     {4096, "high voltage disconnect"},
     {8192, "undefined"},
     {16384, "system miswire"},
     {32768, "MOSFET open"},
     {65536, "P12 voltage off"},
     {131072, "high input voltage current limit"},
     {262144, "ADC input max"},
     {524288, "controller was reset"}
     };

     struct status charge_states[] = {

     {0, "START"},
     {1, "NIGHT_CHECK"},
     {2, "DISCONNECT"},
     {3, "NIGHT"},
     {4, "FAULT"},
     {5, "MPPT"},
     {6, "ABSORPTION"},
     {7, "FLOAT"},
     {8, "EQUALIZE"},
     {9, "SLAVE"}
     };
     */

    blast_info("Attempting to connect to charge controller.");

    while (ctlr->req_state != CC_STATE_SHUTDOWN) {
        if (ctlr->req_state == CC_STATE_READY && ctlr->state != CC_STATE_READY) {
            if (ctlr->mb) {
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
                usleep(1000000);
                continue;
            }
            ctlr->state = CC_STATE_READY;
            have_warned_connect = 0;
        }

        if (ctlr->state != CC_STATE_READY) {
            blast_info("Charge controller %d not ready!  Sleeping...", ctlr->id);
            usleep(1000000);
            continue;
        }
        ctlr->has_error = 0;
        /* retrieve voltage and current scaling factors (needed to turn the
         ADC output into meaningful V & I values) from register
         addresses 1-4 */

        blast_dbg("Reading Scale Factors");
        if ((scale.num = modbus_read_registers(ctlr->mb, 1, 4, scale.arr)) < 0) {
            blast_err("Could not read scaling factors for Modbus"
                    " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
            ctlr->has_error = 1;
        }
        blast_dbg("Voltage Scaling Factor: 0x%04X:0x%04X", scale.arr[0], scale.arr[1]);
        blast_dbg("Current Scaling Factor: 0x%04X:0x%04X", scale.arr[2], scale.arr[3]);

        /* poll charge controller for battery and array voltages and currents,
         which range from register addresses [26 or]27-30 */

        if ((elec.num = modbus_read_registers(ctlr->mb, 26, 5, elec.arr)) < 0) {
            blast_err("Could not read voltages for Modbus"
                    " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
            ctlr->has_error = 1;
        }

        /* heatsink temperature in degrees C (addr 36) */

        if ((temp.num = modbus_read_registers(ctlr->mb, 36, 1, temp.arr)) < 0) {
            blast_err("Could not read heatsink temp for Modbus"
                    " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
            ctlr->has_error = 1;
        }

        /* charge controller fault bitfield (addr 45) */

        if ((fault.num = modbus_read_registers(ctlr->mb, 45, 1, fault.arr)) < 0) {
            blast_err("Could not read fault bitfield for Modbus"
                    " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
        }

        /* charge controller alarm bitfield (spans 2 regs with addrs 47,48) */

        if ((alarm.num = modbus_read_registers(ctlr->mb, 47, 2, alarm.arr)) < 0) {
            blast_err("Could not read alarm bitfield for Modbus"
                    " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
            ctlr->has_error = 1;
        }

        /* controller LED state, charge state and target charging voltage (addrs 50, 51, 52) */

        if ((charge.num = modbus_read_registers(ctlr->mb, 50, 3, charge.arr)) < 0) {
            blast_err("Could not read state vars for Modbus"
                    " charge controller at %s: %s", ctlr->addr, modbus_strerror(errno));
            ctlr->has_error = 1;
        }

//        if (ctlr->has_error) {
//            ctlr->req_state = CC_STATE_RESET;
//
//            /* put charge controller data values into obvious error state */
//            ctlr->V_batt = 0.0;
//            ctlr->V_arr = 0.0;
//            ctlr->I_batt = 0.0;
//            ctlr->I_arr = 0.0;
//            ctlr->V_targ = 0.0;
//            ctlr->T_hs = 0;
//            ctlr->charge_state = 10;
//            ctlr->led_state = 18;
//            usleep(1000000);
//            continue; // go back up to top of infinite loop
//        }

        /* compute values of things that need scaling */

        float Vscale, Iscale;
        Vscale = *scale.arr + (*(scale.arr + 1)) / 65536.0;
        Iscale = *(scale.arr + 2) + (*(scale.arr + 3)) / 65536.0;

        /* *elec.arr is the battery voltage at the charging terminals
         (less accurate) as opposed to the sense terminals.
         EDIT: actually these terminals are shorted, so it doesn't
         matter which reading is used. */

        ctlr->V_batt = *(elec.arr + 1) * Vscale / 32768.0;
        ctlr->V_arr = *(elec.arr + 2) * Vscale / 32768.0;
        ctlr->I_batt = *(elec.arr + 3) * Iscale / 32768.0;
        ctlr->I_arr = *(elec.arr + 4) * Iscale / 32768.0;

        ctlr->V_targ = *(charge.arr + 2) * Vscale / 32768.0;

        ctlr->T_hs = *temp.arr;
        ctlr->fault_field = *fault.arr;

        ctlr->alarm_field_hi = *alarm.arr;
        ctlr->alarm_field_lo = *(alarm.arr + 1);

        ctlr->led_state = *charge.arr;
        ctlr->charge_state = *(charge.arr + 1);

        blast_dbg("Voltage Scaling Factor: %.3f \n", Vscale);
        blast_dbg("Current Scaling Factor: %.3f \n \n", Iscale);
        blast_dbg("Battery Sense Voltage: %.3f V \n", ctlr->V_batt);
        blast_dbg("Solar Array Input Voltage: %.3f V \n", ctlr->V_arr);
        blast_dbg("Output Current to Battery: %.3f A \n", ctlr->I_batt);
        blast_dbg("Input Current from Array: %.3f A \n", ctlr->I_arr);
        blast_dbg("Target Battery Charging Voltage: %.3f V \n \n", ctlr->V_targ);
        blast_dbg("Heatsink Temperature: %d C \n \n", ctlr->T_hs);

//        for (int i = 0; i < nfaults ; i++) {
//
//        if (ctlr->fault_field & fault_bits[i].flag)
//            blast_dbg("FAULT: %s \n", fault_bits[i].message);
//        }

//            parse alarm bitfield

//            if (!ctlr->alarm_field_hi)
//                blast_dbg("No alarm conditions detected \n");
//
//            for (int j = 0; j < nalarms ; j++) {
//
//            if (ctlr->alarm_field & alarm_bits[j].flag)
//                blast_dbg("ALARM: %s \n", alarm_bits[j].message);
//            }
//
//            determine charging state from lookup table

//            printf("Charging State: %s \n \n", charge_states[ctlr->charge_state].message);

        usleep(1000000);
    }
    return NULL;
}
